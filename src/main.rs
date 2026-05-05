//! RoboControl
//! Control servos and ESCs with a PCA9685
use std::io;
use std::thread;
use std::time::{Duration, Instant};
use std::sync::mpsc;

use anyhow::{Context, Result};
use crossterm::{
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
    controls: Controls
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
        Ok(Self { i2c: raw.i2c, pwm: raw.pwm, mav: raw.mav, channel: channels, controls: raw.controls, })
    }
}

fn convert(config_vector: Vec<RawChannelConfig>) -> Result<[Option<ChannelConfig>; NUMBER_OF_CHANNELS]> {
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

struct MavLinkService {
    connection: Connection<MavMessage>,
}


// In MavLinkService, spawn the recv thread and return a receiver
impl MavLinkService {
    fn new(state: &AppConfig) -> Result<Self> {
        let connection = mavlink::connect::<MavMessage>(
            format!("udpin:0.0.0.0:{}", state.mav.port).as_str()
        )?;
        Ok(Self { connection })
    }

    fn send_heart_beat(&self) -> Result<()> {
       let heartbeat = MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
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

struct PwmDriver { pca: Pca9685<I2cdev> }

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
        self.pca.set_channel_on_off(channel, 0, pulse)
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
            match state.channel[c] {
                Some(ch) => self.set_pulse(ch.pwm_channel, ch.neutral)?,
                None => (),
            };
        }
        Ok(())
    }
}

fn render_ui(state: &AppConfig, watchdog: bool) -> Result<()> {
    use io::Write;
    use crossterm::{cursor::MoveTo, terminal::{Clear, ClearType}};
    let mut stdout = io::stdout();
    execute!(stdout, MoveTo(0, 0), Clear(ClearType::All))?;
    println!("*** ROBO CONTROLLER ***");
    println!(" Listening for MavLink Commands");
    println!(" Ctrl+C to Quit");
    println!("");
    if watchdog {
        println!("*** NO SIGNAL DETECTED ***");
        println!("");
    }
    println!("** Current **");
    for c in 0..NUMBER_OF_CHANNELS {
            match state.channel[c] {
                Some(ch) => println!("Channel {}: {}", c, ch.current_value),
                None => (),
            };
    } 
    stdout.flush()?;
    Ok(())
}

fn arm_esc(driver: &mut PwmDriver, state: &AppConfig) -> Result<()> {
    println!("Arming ESC — sending neutral for 2 s…");

    for c in 0..NUMBER_OF_CHANNELS {
            match state.channel[c] {
                Some(ch) => driver.set_pulse(ch.pwm_channel, ch.neutral)?,
                None => (),
            };
    } 
    thread::sleep(Duration::from_secs(2));
    println!("ESCs armed.");
    thread::sleep(Duration::from_millis(500));
    Ok(())
}

fn main() -> Result<()> {
    env_logger::init();

    let mut state = load_configuration()?;

    let mut driver = PwmDriver::new(&state.i2c.path, state.i2c.address, state.pwm.prescale).context("Failed to initialise PCA9685")?;

    let mav : MavLinkService = MavLinkService::new(&state)?;

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

fn translate_message(msg :MavMessage, state: &AppConfig) -> Vec<AbsoluteControlOutput> {
    let mut outputs: Vec<AbsoluteControlOutput> = Vec::new();
    
    let throttle_channel_config= state.channel[state.controls.throttle as usize];

    let steering_channel_config= state.channel[state.controls.steering as usize];
    
    match msg {
        MavMessage::MANUAL_CONTROL(data) => {
            if let Some(throttle_cfg) = throttle_channel_config {
                outputs.push(AbsoluteControlOutput{
                    channel: throttle_cfg.pwm_channel,
                    value: mavlink_to_pwm(data.x, &throttle_cfg),
                });
            }
            
            if let Some(steering_cfg) = steering_channel_config {
                // Steering can be roll or yaw
                let steering = if data.y != 0 {
                    mavlink_to_pwm(data.y, &steering_cfg)
                } else {
                    mavlink_to_pwm(data.r, &steering_cfg)
                };

                outputs.push(AbsoluteControlOutput{
                    channel: steering_cfg.pwm_channel,
                    value: steering,
                });
            }
        }
 
        MavMessage::RC_CHANNELS_OVERRIDE(data) => {
            eprintln!("RC override: ch1={} ch2={} ch3={}", 
                data.chan1_raw, data.chan2_raw, data.chan3_raw);

            // 18 available channels.
            let channels: [u16; 8] = [
                data.chan1_raw, data.chan2_raw, data.chan3_raw, data.chan4_raw,
                data.chan5_raw, data.chan6_raw, data.chan7_raw, data.chan8_raw,
            ];

            for i in 0..channels.len() {

                if channels[i] == 0 || channels[i] == 65535 {
                    continue;
                }

                let channel_config: Option<&ChannelConfig> = state.channel.iter()
                    .find(|cfg| cfg.map_or(false, |c| c.mavlink_channel as usize == i+1))
                    .and_then(|cfg| cfg.as_ref());

                // Convert
                let ichan: i16 = (channels[i] as i32 - 1500).clamp(-1000, 1000) as i16;

                if let Some(cfg) = channel_config {
                    outputs.push(AbsoluteControlOutput { channel: cfg.pwm_channel, value: mavlink_to_pwm(ichan, cfg) });
                }
            }
        }
 
        MavMessage::SET_ACTUATOR_CONTROL_TARGET(data) => {
            // Only handle group 0 (primary flight control group).
            if data.group_mlx == 0 {
                if let Some(throttle_cfg) = throttle_channel_config {
                    outputs.push(AbsoluteControlOutput{
                        channel: throttle_cfg.pwm_channel,
                        value: mavlink_to_pwm((data.controls[3] * 1000.0) as i16, &throttle_cfg),
                    });
                }

                if let Some(steering_cfg) = steering_channel_config {
                    outputs.push(AbsoluteControlOutput{
                        channel: steering_cfg.pwm_channel,
                        value: mavlink_to_pwm((data.controls[0] * 1000.0) as i16, &steering_cfg),
                    });                

                }
            }
        }
 
        // Ignore other messages
        _ => {
            eprintln!("Unhandled message: {:?}", msg);
        },
    }

    outputs
}

const MESSAGE_TIMEOUT: Duration = Duration::from_millis(500);

fn run_loop(
    driver: &mut PwmDriver,
    state: &mut AppConfig,
    mavlink_service: MavLinkService,
) -> Result<()> {
    let (tx, rx) = mpsc::channel::<MavMessage>();
    let mut time_since_last_heartbeat = Instant::now();
    let mut time_since_last_message: Option<Instant> = None;
    let mut watchdog_triggered = false;

    thread::scope(|s| {
        s.spawn(|| {
            loop {
                match mavlink_service.connection.recv() {
                    Ok((_header, msg)) => {
                        if tx.send(msg).is_err() {
                            break;
                        }
                    }
                    Err(mavlink::error::MessageReadError::Io(e))
                        if e.kind() == std::io::ErrorKind::WouldBlock => {
                            // This shouldn't happen, but it's no big deal if it does
                        }
                    Err(e) => {
                        eprintln!("MAVLink recv error: {e:?}");
                        break;
                    }
                }
            }
        });

        // Main loop runs in the scope too, so the scope (and the thread) 
        // can't outlive mavlink_service
        loop {
            if time_since_last_heartbeat.elapsed() > HEARTBEAT_DURATION {
                mavlink_service.send_heart_beat()?;
                time_since_last_heartbeat = Instant::now();
            }

            if let Some(t) = time_since_last_message {
                if t.elapsed() > MESSAGE_TIMEOUT {
                    if !watchdog_triggered {
                        eprintln!("WARNING: No MAVLink message for {}ms — going neutral",
                            MESSAGE_TIMEOUT.as_millis());
                        driver.safe_stop(state)?;
                        watchdog_triggered = true;
                    }
                } else {
                    watchdog_triggered = false;
                }
            }

            match rx.recv_timeout(HEARTBEAT_DURATION) {
                Ok(msg) => {
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
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    eprintln!("MAVLink recv thread died — stopping");
                    driver.safe_stop(state)?;
                    break;
                }
            }
            render_ui(state, watchdog_triggered)?;
        }

        Ok(())
    })
}