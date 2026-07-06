//! RoboControl
//! Control servos and ESCs with a PCA9685
use std::thread;
use std::time::{Duration, Instant};
use std::sync::mpsc;

use anyhow::{Context, Result};
use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Address, Channel, Pca9685};
use config::Config;
use mavlink::common::MavMessage;
use mavlink::MavConnection;
use mavlink::Connection;
use serde::Deserialize;


const NUMBER_OF_CHANNELS: usize = 16;

const HEARTBEAT_DURATION: Duration = Duration::from_secs(1);

// The control loop should not update more than 50Hz to avoid confusing the servos
const CONTROL_LOOP_MIN_DURATION: Duration = Duration::from_millis(20);

#[derive(Debug, Clone, Copy)]

// ── Config types ──────────────────────────────────────────────────────────────

#[derive(Debug, Deserialize)]
struct I2cConfig {
    address: u8,
    path: String,
}

#[derive(Debug, Deserialize)]
struct PwmConfig {
    prescale: u8,
}

#[derive(Debug, Deserialize)]
struct MavlinkConfig {
    port: u16,
}

#[derive(Debug, Deserialize)]
struct RawChannelConfig {
    pwm_channel: u8,
    min: u16,
    max: u16,
    neutral: u16,
    mavlink_channel: u8,
    max_step: u16,
}

#[derive(Debug, Deserialize)]
struct RawConfig {
    i2c: I2cConfig,
    pwm: PwmConfig,
    mav: MavlinkConfig,
    channel: Vec<RawChannelConfig>,
    controls: Controls,
}

#[derive(Copy, Clone)]
struct ChannelConfig {
    pwm_channel: Channel,
    min: u16,
    max: u16,
    neutral: u16,
    mavlink_channel: u8,
    current_value: u16,
    changed: bool,
    max_step: u16,
}

struct AbsoluteControlOutput {
    channel: Channel,
    value: u16,
}

#[derive(Debug, Deserialize)]
struct Controls {
    steering: u8,
    throttle: u8,
}

struct AppConfig {
    i2c: I2cConfig,
    pwm: PwmConfig,
    mav: MavlinkConfig,
    channel: [Option<ChannelConfig>; NUMBER_OF_CHANNELS],
    controls: Controls,
}

impl AppConfig {
    fn from_raw(raw: RawConfig) -> Result<Self> {
        let channels: [Option<ChannelConfig>; 16] = convert(raw.channel)?;
        Ok(Self {
            i2c: raw.i2c,
            pwm: raw.pwm,
            mav: raw.mav,
            channel: channels,
            controls: raw.controls,
        })
    }
}

fn convert(
    config_vector: Vec<RawChannelConfig>,
) -> Result<[Option<ChannelConfig>; NUMBER_OF_CHANNELS]> {
    let mut result: [Option<ChannelConfig>; NUMBER_OF_CHANNELS] = [None; NUMBER_OF_CHANNELS];
    for (i, val) in config_vector.into_iter().enumerate() {
        result[i] = Some(val.try_into().expect("channel invalid"));
    }
    Ok(result)
}

impl TryFrom<RawChannelConfig> for ChannelConfig {
    type Error = String;

    fn try_from(raw: RawChannelConfig) -> Result<Self, Self::Error> {
        Ok(Self {
            pwm_channel: channel_from_u8(raw.pwm_channel)
                .ok_or(format!("Invalid pwm_channel: {}", raw.pwm_channel))?,
            min: raw.min,
            max: raw.max,
            neutral: raw.neutral,
            mavlink_channel: raw.mavlink_channel,
            current_value: raw.neutral,
            changed: true,
            max_step: raw.max_step,
        })
    }
}

// ── MAVLink service ───────────────────────────────────────────────────────────

struct MavLinkService {
    connection: Connection<MavMessage>,
}

impl MavLinkService {
    fn new(state: &AppConfig) -> Result<Self> {
        let mut connection = mavlink::connect::<MavMessage>(
            format!("udpin:0.0.0.0:{}", state.mav.port).as_str(),
        )?;
        connection.set_protocol_version(mavlink::MavlinkVersion::V2);
        connection.set_allow_recv_any_version(true);
        Ok(Self { connection })
    }

    fn send_heart_beat(&self) -> Result<()> {
        let heartbeat =
            MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: mavlink::common::MavType::MAV_TYPE_GROUND_ROVER,
                autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
                base_mode: mavlink::common::MavModeFlag::empty(),
                system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
                mavlink_version: 0xFD,
            });
        self.connection.send(&mavlink::MavHeader::default(), &heartbeat)?;
        Ok(())
    }
}

// ── PWM driver ────────────────────────────────────────────────────────────────

struct PwmDriver {
    pca: Pca9685<I2cdev>,
}

impl PwmDriver {
    fn new(ic2_bus_path: &str, ic2_address: u8, prescale: u8) -> Result<Self> {
        let i2c = I2cdev::new(ic2_bus_path)
            .with_context(|| format!("Failed to open I2C bus '{ic2_bus_path}'"))?;
        let address = Address::from(ic2_address);
        let mut pca = Pca9685::new(i2c, address)
            .map_err(|e| anyhow::anyhow!("PCA9685 init error: {:?}", e))?;
        pca.set_prescale(prescale)
            .map_err(|e| anyhow::anyhow!("set_prescale error: {:?}", e))?;
        pca.enable()
            .map_err(|e| anyhow::anyhow!("enable error: {:?}", e))?;
        Ok(Self { pca })
    }

    fn set_pulse(&mut self, channel: Channel, pulse: u16) -> Result<()> {
        self.pca
            .set_channel_on_off(channel, 0, pulse)
            .map_err(|e| anyhow::anyhow!("set_channel_on_off error: {:?}", e))
    }

    fn apply(&mut self, state: &mut AppConfig) -> Result<()> {
        for c in 0..NUMBER_OF_CHANNELS {
            if let Some(ch) = &mut state.channel[c] {
                if ch.changed {
                    self.set_pulse(ch.pwm_channel, ch.current_value)?;
                    ch.changed = false;
                }
            }
        }
        Ok(())
    }

    fn safe_stop(&mut self, state: &mut AppConfig) -> Result<()> {
        for c in 0..NUMBER_OF_CHANNELS {
            if let Some(cfg) = &mut state.channel[c] {
                cfg.current_value = cfg.neutral;
                cfg.changed = true;
                break;
            }
        }

        self.apply(state)?;
        Ok(())
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn arm_esc(driver: &mut PwmDriver, state: &AppConfig) -> Result<()> {
    println!("Arming ESC — sending neutral for 2 s…");
    for c in 0..NUMBER_OF_CHANNELS {
        if let Some(ch) = state.channel[c] {
            driver.set_pulse(ch.pwm_channel, ch.neutral)?;
        }
    }
    thread::sleep(Duration::from_secs(2));
    println!("ESCs armed.");
    thread::sleep(Duration::from_millis(500));
    Ok(())
}

fn load_configuration() -> Result<AppConfig> {
    let raw: RawConfig = Config::builder()
        .add_source(config::File::with_name("config"))
        .add_source(config::Environment::with_prefix("ROBOCONTROL"))
        .build()
        .map_err(|e| anyhow::anyhow!("config build error: {:?}", e))?
        .try_deserialize()?;
    AppConfig::from_raw(raw)
}

fn channel_from_u8(n: u8) -> Option<Channel> {
    match n {
        0  => Some(Channel::C0),
        1  => Some(Channel::C1),
        2  => Some(Channel::C2),
        3  => Some(Channel::C3),
        4  => Some(Channel::C4),
        5  => Some(Channel::C5),
        6  => Some(Channel::C6),
        7  => Some(Channel::C7),
        8  => Some(Channel::C8),
        9  => Some(Channel::C9),
        10 => Some(Channel::C10),
        11 => Some(Channel::C11),
        12 => Some(Channel::C12),
        13 => Some(Channel::C13),
        14 => Some(Channel::C14),
        15 => Some(Channel::C15),
        _  => None,
    }
}

fn mavlink_raw_to_pwm(input: u16, cfg: &ChannelConfig) -> u16 {
    const RAW_MAX: u16 = 2000;
    const RAW_MIN: u16 = 1000;
    
    if input == 0 || input == u16::MAX {
        return mavlink_scaled_to_pwm(i16::MAX, cfg);
    }

    let clamped_input = input.clamp(RAW_MIN, RAW_MAX);

    let scaled_value: i16 = ((((clamped_input - RAW_MIN) as f32 / (RAW_MAX - RAW_MIN) as f32  ) * 2.0 - 1.0) * 10000.0).round() as i16;

    return mavlink_scaled_to_pwm(scaled_value, cfg);
}

fn mavlink_scaled_to_pwm(input: i16, cfg: &ChannelConfig) -> u16 {
    if input == i16::MAX {
        return cfg.neutral;
    }

    let input = input.clamp(-10000, 10000);
    if input >= 0 {
        let range = (cfg.max - cfg.neutral) as f32;
        return cfg.neutral + ((input as f32 / 10000.0) * range).round() as u16;
    }
    let range = (cfg.neutral - cfg.min) as f32;
    cfg.neutral - (((-input) as f32 / 10000.0) * range).round() as u16
}


fn translate_message(msg: MavMessage, state: &AppConfig) -> Vec<AbsoluteControlOutput> {
    let mut outputs: Vec<AbsoluteControlOutput> = Vec::new();
    let throttle_channel_config = state.channel[state.controls.throttle as usize];
    let steering_channel_config = state.channel[state.controls.steering as usize];

    match msg {
        MavMessage::MANUAL_CONTROL(data) => {
            if let Some(throttle_cfg) = throttle_channel_config {
                outputs.push(AbsoluteControlOutput {
                    channel: throttle_cfg.pwm_channel,
                    value: mavlink_scaled_to_pwm(data.x, &throttle_cfg),
                });
            }
            if let Some(steering_cfg) = steering_channel_config {
                let steering = if data.y != 0 {
                    mavlink_scaled_to_pwm(data.y, &steering_cfg)
                } else {
                    mavlink_scaled_to_pwm(data.r, &steering_cfg)
                };
                outputs.push(AbsoluteControlOutput {
                    channel: steering_cfg.pwm_channel,
                    value: steering,
                });
            }
        }

        MavMessage::RC_CHANNELS_OVERRIDE(data) => {
            let channels: [u16; 8] = [
                data.chan1_raw, data.chan2_raw, data.chan3_raw, data.chan4_raw,
                data.chan5_raw, data.chan6_raw, data.chan7_raw, data.chan8_raw,
            ];
            for i in 0..channels.len() {
                if channels[i] == 0 || channels[i] == u16::MAX {
                    continue;
                }
                let channel_config: Option<&ChannelConfig> = state
                    .channel
                    .iter()
                    .find(|cfg| cfg.map_or(false, |c| c.mavlink_channel as usize == i + 1))
                    .and_then(|cfg| cfg.as_ref());
                if let Some(cfg) = channel_config {
                    outputs.push(AbsoluteControlOutput {
                        channel: cfg.pwm_channel,
                        value: mavlink_raw_to_pwm(channels[i], cfg),
                    });
                }
            }
        }

        MavMessage::SET_ACTUATOR_CONTROL_TARGET(data) => {
            if data.group_mlx == 0 {
                if let Some(throttle_cfg) = throttle_channel_config {
                    outputs.push(AbsoluteControlOutput {
                        channel: throttle_cfg.pwm_channel,
                        value: mavlink_scaled_to_pwm(
                            (data.controls[3] * 10000.0) as i16,
                            &throttle_cfg,
                        ),
                    });
                }
                if let Some(steering_cfg) = steering_channel_config {
                    outputs.push(AbsoluteControlOutput {
                        channel: steering_cfg.pwm_channel,
                        value: mavlink_scaled_to_pwm(
                            (data.controls[0] * 10000.0) as i16,
                            &steering_cfg,
                        ),
                    });
                }
            }
        }

        _ => {
            eprintln!("Unhandled message: {:?}", msg);
        }
    }
    outputs
}

/// Clamp a PWM value inside the channel's [min, max] range.
fn clamp_to_channel(value: i32, cfg: &ChannelConfig) -> u16 {
    value.clamp(cfg.min as i32, cfg.max as i32) as u16
}

/// Reset a single channel to neutral.
fn reset_channel(state: &mut AppConfig, channel_idx: u8) {
    if let Some(cfg) = &mut state.channel[channel_idx as usize] {
        cfg.current_value = cfg.neutral;
        cfg.changed = true;
    }
}

fn slew(current: u16, target: u16, max_step: u16) -> u16 {
    if target > current {
        (current + max_step).min(target)
    } else {
        (current - max_step).max(target)
    }    
}

// ── Main loop ─────────────────────────────────────────────────────────────────

const MESSAGE_TIMEOUT: Duration = Duration::from_millis(500);

/// Combined event for the main select loop.
enum LoopEvent {
    Mav(MavMessage),
}

fn run_loop(
    driver: &mut PwmDriver,
    state: &mut AppConfig,
    mavlink_service: MavLinkService,
) -> Result<()> {
    // Control channels funnelled into one via LoopEvent
    let (mav_tx, mav_rx) = mpsc::channel::<MavMessage>();

    let mut time_since_last_heartbeat = Instant::now();
    let mut time_since_last_message: Option<Instant> = None;
    let mut watchdog_triggered = false;
    let mut time_since_last_update: Instant = Instant::now();

    thread::scope(|s| {
        // MAVLink receiver thread
        s.spawn(|| {
            loop {

                match mavlink_service.connection.recv() {
                    Ok((_header, msg)) => {
                        if mav_tx.send(msg).is_err() {
                            break;
                        }
                    }
                    Err(mavlink::error::MessageReadError::Io(e))
                        if e.kind() == std::io::ErrorKind::WouldBlock => {}
                    Err(e) => {
                        eprintln!("MAVLink recv error: {e:?}");
                        break;
                    }
                }
            }
        });

        // Main control loop
        loop {

            // TODO: Refactor the control loop to call an "update hardware" function instead of driver.apply directly.
            // Have the hardware update at the end of the loop, only if >= min update time, then reset last update time
            // Then, remove the below sleep and update polling to be faster.

            // Don't update too fast for the sake of the servos
            if time_since_last_update.elapsed() < CONTROL_LOOP_MIN_DURATION {
                thread::sleep(CONTROL_LOOP_MIN_DURATION - time_since_last_update.elapsed());
                time_since_last_update = Instant::now();
            }

            // ── Heartbeat ────────────────────────────────────────────────────
            if time_since_last_heartbeat.elapsed() > HEARTBEAT_DURATION {
                mavlink_service.send_heart_beat()?;
                time_since_last_heartbeat = Instant::now();
            }

            // ── Watchdog ─────────────────────────────────────────────────────
            if let Some(t) = time_since_last_message {
                if t.elapsed() > MESSAGE_TIMEOUT {
                    if !watchdog_triggered {
                        eprintln!(
                            "WARNING: No MAVLink message for {}ms — going neutral",
                            MESSAGE_TIMEOUT.as_millis()
                        );
                        driver.safe_stop(state)?;
                        watchdog_triggered = true;
                    }
                } else {
                    watchdog_triggered = false;
                }
            }

            let poll_timeout = CONTROL_LOOP_MIN_DURATION;

            let mut events: Vec<LoopEvent> = Vec::new();

            // Drain all pending MAVLink messages
            loop {
                match mav_rx.recv_timeout(poll_timeout) {
                    Ok(msg) => events.push(LoopEvent::Mav(msg)),
                    Err(mpsc::RecvTimeoutError::Timeout) => break,
                    Err(mpsc::RecvTimeoutError::Disconnected) => {
                        eprintln!("MAVLink recv thread died — stopping");
                        driver.safe_stop(state)?;
                        return Ok(());
                    }
                }
                // Only block on the very first recv; drain the rest instantly
            }

            // ── Process events ────────────────────────────────────────────
            for ev in events {
                match ev {
                    LoopEvent::Mav(msg) => {
                        time_since_last_message = Some(Instant::now());
                        let commands = translate_message(msg, state);
                        for command in commands {
                            for c in 0..NUMBER_OF_CHANNELS {
                                if let Some(cfg) = &mut state.channel[c] {
                                    if cfg.pwm_channel == command.channel {
                                        cfg.current_value = slew(cfg.current_value, command.value, cfg.max_step);
                                        cfg.changed = true;
                                        break;
                                    }
                                }
                            }
                        }
                        driver.apply(state)?;
                    }
                }
            }
        }

        Ok(())
    })
}

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    env_logger::init();

    let mut state = load_configuration()?;
    let mut driver = PwmDriver::new(
        &state.i2c.path,
        state.i2c.address,
        state.pwm.prescale,
    )
    .context("Failed to initialise PCA9685")?;

    let mav = MavLinkService::new(&state)?;

    arm_esc(&mut driver, &state)?;

    let result = run_loop(&mut driver, &mut state, mav);

    driver.safe_stop(&mut state)?;
    println!("\nRC robo controller stopped. Goodbye!");
    result
}