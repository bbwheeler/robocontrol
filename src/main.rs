//! RoboControl
//! Control servos and ESCs with a PCA9685
use std::io;
use std::thread;
use std::time::{Duration, Instant};
use std::sync::mpsc;

use anyhow::{Context, Result};
use crossterm::{
    event::{self, Event, KeyCode, KeyEvent, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode},
    execute,
    cursor,
};
use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Address, Channel, Pca9685};
use config::Config;
use mavlink::ardupilotmega::MavMessage;
use mavlink::MavConnection;
use mavlink::Connection;
use serde::Deserialize;


const NUMBER_OF_CHANNELS: usize = 16;

const HEARTBEAT_DURATION: Duration = Duration::from_secs(1);

/// How much a single keypress moves the PWM value (in raw PWM ticks).
/// Tune this to taste — 10 ticks is roughly 1 % of a typical 1000-tick range.
const KEY_STEP: i32 = 10;

// ── Key events sent from the keyboard thread ─────────────────────────────────

#[derive(Debug, Clone, Copy)]
enum KeyCommand {
    ThrottleUp,
    ThrottleDown,
    SteerLeft,
    SteerRight,
    Neutral,   // space — instant safe-stop
    Quit,
}

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
        })
    }
}

// ── MAVLink service ───────────────────────────────────────────────────────────

struct MavLinkService {
    connection: Connection<MavMessage>,
}

impl MavLinkService {
    fn new(state: &AppConfig) -> Result<Self> {
        let connection = mavlink::connect::<MavMessage>(
            format!("udpin:0.0.0.0:{}", state.mav.port).as_str(),
        )?;
        Ok(Self { connection })
    }

    fn send_heart_beat(&self) -> Result<()> {
        let heartbeat =
            MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_GROUND_ROVER,
                autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
                system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
                mavlink_version: 0x3,
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

    fn safe_stop(&mut self, state: &AppConfig) -> Result<()> {
        for c in 0..NUMBER_OF_CHANNELS {
            if let Some(ch) = state.channel[c] {
                self.set_pulse(ch.pwm_channel, ch.neutral)?;
            }
        }
        Ok(())
    }
}

// ── UI ────────────────────────────────────────────────────────────────────────

fn render_ui(state: &AppConfig, watchdog: bool) -> Result<()> {
    use io::Write;
    use crossterm::{cursor::MoveTo, terminal::{Clear, ClearType}};
    let mut stdout = io::stdout();
    execute!(stdout, MoveTo(0, 0), Clear(ClearType::All))?;
    println!("*** ROBO CONTROLLER ***");
    println!(" Listening for MavLink Commands");
    println!(" Arrow keys: throttle (↑↓) / steer (←→)  |  Space: neutral  |  Q: quit");
    println!();
    if watchdog {
        println!("*** NO SIGNAL DETECTED ***");
        println!();
    }
    println!("** Current **");
    for c in 0..NUMBER_OF_CHANNELS {
        if let Some(ch) = state.channel[c] {
            println!("Channel {}: {}", c, ch.current_value);
        }
    }
    stdout.flush()?;
    Ok(())
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

fn mavlink_to_pwm(input: i16, cfg: &ChannelConfig) -> u16 {
    let input = input.clamp(-1000, 1000);
    if input == 0 {
        return cfg.neutral;
    }
    if input > 0 {
        let range = (cfg.max - cfg.neutral) as f32;
        return cfg.neutral + ((input as f32 / 1000.0) * range).round() as u16;
    }
    let range = (cfg.neutral - cfg.min) as f32;
    cfg.neutral - (((-input) as f32 / 1000.0) * range).round() as u16
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
                    value: mavlink_to_pwm(data.x, &throttle_cfg),
                });
            }
            if let Some(steering_cfg) = steering_channel_config {
                let steering = if data.y != 0 {
                    mavlink_to_pwm(data.y, &steering_cfg)
                } else {
                    mavlink_to_pwm(data.r, &steering_cfg)
                };
                outputs.push(AbsoluteControlOutput {
                    channel: steering_cfg.pwm_channel,
                    value: steering,
                });
            }
        }

        MavMessage::RC_CHANNELS_OVERRIDE(data) => {
            eprintln!(
                "RC override: ch1={} ch2={} ch3={}",
                data.chan1_raw, data.chan2_raw, data.chan3_raw
            );
            let channels: [u16; 8] = [
                data.chan1_raw, data.chan2_raw, data.chan3_raw, data.chan4_raw,
                data.chan5_raw, data.chan6_raw, data.chan7_raw, data.chan8_raw,
            ];
            for i in 0..channels.len() {
                if channels[i] == 0 || channels[i] == 65535 {
                    continue;
                }
                let channel_config: Option<&ChannelConfig> = state
                    .channel
                    .iter()
                    .find(|cfg| cfg.map_or(false, |c| c.mavlink_channel as usize == i + 1))
                    .and_then(|cfg| cfg.as_ref());
                let ichan: i16 =
                    (channels[i] as i32 - 1500).clamp(-1000, 1000) as i16;
                if let Some(cfg) = channel_config {
                    outputs.push(AbsoluteControlOutput {
                        channel: cfg.pwm_channel,
                        value: mavlink_to_pwm(ichan, cfg),
                    });
                }
            }
        }

        MavMessage::SET_ACTUATOR_CONTROL_TARGET(data) => {
            if data.group_mlx == 0 {
                if let Some(throttle_cfg) = throttle_channel_config {
                    outputs.push(AbsoluteControlOutput {
                        channel: throttle_cfg.pwm_channel,
                        value: mavlink_to_pwm(
                            (data.controls[3] * 1000.0) as i16,
                            &throttle_cfg,
                        ),
                    });
                }
                if let Some(steering_cfg) = steering_channel_config {
                    outputs.push(AbsoluteControlOutput {
                        channel: steering_cfg.pwm_channel,
                        value: mavlink_to_pwm(
                            (data.controls[0] * 1000.0) as i16,
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

// ── Keyboard thread ───────────────────────────────────────────────────────────

/// Spawn a thread that translates crossterm key events into `KeyCommand`s and
/// forwards them over `tx`.  The thread exits when `tx` is dropped (i.e. when
/// the main loop returns), or when the user presses Q / Ctrl-C.
fn spawn_keyboard_thread(tx: mpsc::Sender<KeyCommand>) {
    thread::spawn(move || {
        loop {
            // poll with a short timeout so we don't block forever if tx dies
            match event::poll(Duration::from_millis(50)) {
                Ok(true) => {}
                Ok(false) => continue,
                Err(_) => break,
            }

            let ev = match event::read() {
                Ok(e) => e,
                Err(_) => break,
            };

            let cmd = match ev {
                Event::Key(KeyEvent { code, modifiers, .. }) => match code {
                    KeyCode::Up    => Some(KeyCommand::ThrottleUp),
                    KeyCode::Down  => Some(KeyCommand::ThrottleDown),
                    KeyCode::Left  => Some(KeyCommand::SteerLeft),
                    KeyCode::Right => Some(KeyCommand::SteerRight),
                    KeyCode::Char(' ') => Some(KeyCommand::Neutral),
                    KeyCode::Char('q') | KeyCode::Char('Q') => Some(KeyCommand::Quit),
                    KeyCode::Char('c')
                        if modifiers.contains(KeyModifiers::CONTROL) =>
                    {
                        Some(KeyCommand::Quit)
                    }
                    _ => None,
                },
                _ => None,
            };

            if let Some(cmd) = cmd {
                if tx.send(cmd).is_err() {
                    break; // main loop has exited
                }
            }
        }
    });
}

// ── Apply a KeyCommand to AppConfig ──────────────────────────────────────────

/// Clamp a PWM value inside the channel's [min, max] range.
fn clamp_to_channel(value: i32, cfg: &ChannelConfig) -> u16 {
    value.clamp(cfg.min as i32, cfg.max as i32) as u16
}

/// Apply one keyboard step to a single channel index, returning whether the
/// channel existed.
fn apply_key_step(state: &mut AppConfig, channel_idx: u8, delta: i32) -> bool {
    if let Some(cfg) = &mut state.channel[channel_idx as usize] {
        let next = clamp_to_channel(cfg.current_value as i32 + delta, cfg);
        if next != cfg.current_value {
            cfg.current_value = next;
            cfg.changed = true;
        }
        true
    } else {
        false
    }
}

/// Reset a single channel to neutral.
fn reset_channel(state: &mut AppConfig, channel_idx: u8) {
    if let Some(cfg) = &mut state.channel[channel_idx as usize] {
        cfg.current_value = cfg.neutral;
        cfg.changed = true;
    }
}

// ── Main loop ─────────────────────────────────────────────────────────────────

const MESSAGE_TIMEOUT: Duration = Duration::from_millis(500);

/// Combined event for the main select loop.
enum LoopEvent {
    Mav(MavMessage),
    Key(KeyCommand),
}

fn run_loop(
    driver: &mut PwmDriver,
    state: &mut AppConfig,
    mavlink_service: MavLinkService,
) -> Result<()> {
    // Two channels funnelled into one via LoopEvent
    let (mav_tx, mav_rx) = mpsc::channel::<MavMessage>();
    let (key_tx, key_rx) = mpsc::channel::<KeyCommand>();

    let mut time_since_last_heartbeat = Instant::now();
    let mut time_since_last_message: Option<Instant> = None;
    let mut watchdog_triggered = false;
    let mut quit_requested = false;

    // Spawn the keyboard listener
    spawn_keyboard_thread(key_tx);

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

            // ── Collect events (MAVLink + keyboard) ───────────────────────
            // Use a short timeout so both queues are drained quickly and the
            // heartbeat & watchdog timers stay responsive.
            let poll_timeout = Duration::from_millis(20);

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

            // Drain all pending keyboard events
            loop {
                match key_rx.try_recv() {
                    Ok(cmd) => events.push(LoopEvent::Key(cmd)),
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => break,
                }
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
                                        cfg.current_value = command.value;
                                        cfg.changed = true;
                                        break;
                                    }
                                }
                            }
                        }
                        driver.apply(state)?;
                    }

                    LoopEvent::Key(cmd) => {
                        match cmd {
                            KeyCommand::Quit => {
                                quit_requested = true;
                            }
                            KeyCommand::Neutral => {
                                driver.safe_stop(state)?;
                                // Reset in-memory state to neutral too
                                reset_channel(state, state.controls.throttle);
                                reset_channel(state, state.controls.steering);
                            }
                            KeyCommand::ThrottleUp => {
                                apply_key_step(state, state.controls.throttle, KEY_STEP);
                                driver.apply(state)?;
                            }
                            KeyCommand::ThrottleDown => {
                                apply_key_step(state, state.controls.throttle, -KEY_STEP);
                                driver.apply(state)?;
                            }
                            KeyCommand::SteerLeft => {
                                apply_key_step(state, state.controls.steering, -KEY_STEP);
                                driver.apply(state)?;
                            }
                            KeyCommand::SteerRight => {
                                apply_key_step(state, state.controls.steering, KEY_STEP);
                                driver.apply(state)?;
                            }
                        }
                    }
                }
            }

            if quit_requested {
                driver.safe_stop(state)?;
                break;
            }

            render_ui(state, watchdog_triggered)?;
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
    enable_raw_mode().context("Failed to enable raw terminal mode")?;
    execute!(io::stdout(), cursor::Hide)?;
    render_ui(&state, false)?;

    let result = run_loop(&mut driver, &mut state, mav);

    let _ = disable_raw_mode();
    let _ = execute!(io::stdout(), cursor::Show);
    driver.safe_stop(&state)?;
    println!("\nRC robo controller stopped. Goodbye!");
    result
}