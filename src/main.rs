//! RoboControl
//! Control servos and ESCs with a PCA9685
use std::io;
use std::thread;
use std::cmp;
use std::time::{Duration, Instant};

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
use mavlink::{MavConnection, Connection};
use serde::Deserialize;


const NUMBER_OF_CHANNELS: usize = 16;

const HEARTBEAT_DURATION: Duration = Duration::from_secs(1);

enum Adjustment {
    INCREASE,
    DECREASE,
    MAX,
    MIN,
    NEUTRAL,
}
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
    step: u16,
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
    step: u16,
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
        let mut channels: [Option<ChannelConfig>; NUMBER_OF_CHANNELS] = std::array::from_fn(|_| None);
        for (i, ch) in raw.channel.iter().enumerate() {
            if i < NUMBER_OF_CHANNELS {
                channels[i] = Some(ch.try_into().expect("invalid pwm channel"));
            }
        }


        let channels: [Option<ChannelConfig>; 16] = convert(raw.channel)?;
        Ok(Self { i2c: raw.i2c, pwm: raw.pwm, mav: raw.mav, channel: channels, controls: raw.controls, })
    }

    fn adjust(&mut self, chan: usize, adjustment: Adjustment) {
        match &mut self.channel[chan] {
            Some(ch) => {
                match adjustment {
                    Adjustment::INCREASE => ch.current_value = cmp::min(ch.current_value + ch.step, ch.max),
                    Adjustment::DECREASE => ch.current_value = cmp::max(ch.current_value - ch.step, ch.min),
                    Adjustment::MAX => ch.current_value = ch.max,
                    Adjustment::MIN => ch.current_value = ch.min,
                    Adjustment::NEUTRAL => ch.current_value = ch.neutral,
                }
                ch.changed = true
            },
            None => (),
        };
    }
}

fn convert(config_vector: Vec<RawChannelConfig>) -> Result<[Option<ChannelConfig>; NUMBER_OF_CHANNELS]> {
        let raw_iter = config_vector.iter();

        let mut result: [Option<ChannelConfig>; NUMBER_OF_CHANNELS] = [None; NUMBER_OF_CHANNELS];

        for (i, val) in raw_iter.enumerate() {
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
            step: raw.step,
            mavlink_channel: raw.mavlink_channel,
            current_value: raw.neutral,
            changed: true,
        })
    }
}

impl TryFrom<&RawChannelConfig> for ChannelConfig {
    type Error = String;

    fn try_from(raw: &RawChannelConfig) -> Result<Self, Self::Error> {
        Ok(Self {
            pwm_channel: channel_from_u8(raw.pwm_channel)
                .ok_or(format!("Invalid pwm_channel: {}", raw.pwm_channel))?,
            min: raw.min,
            max: raw.max,
            neutral: raw.neutral,
            step: raw.step,
            mavlink_channel: raw.mavlink_channel,
            current_value: raw.neutral,
            changed: true,
        })
    }
}

struct MavLinkService {
    connection: Connection<mavlink::ardupilotmega::MavMessage>,
}

impl MavLinkService {
    fn new(state: AppConfig) -> Result<Self> {
        let connection = mavlink::connect::<MavMessage>(format!("udpin:0.0.0.0:{}", state.mav.port))?;
        Ok(Self{connection})
    }

    fn send_heart_beat(&self) -> Result<()> {
        let heartbeat = MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_ROVER,
            autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
            system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
            mavlink_version: 0x3,
        });


        self.connection.send(&mavlink::MavHeader::default(), &heartbeat);
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
            match &mut state.channel[c] {
                Some(ch) => {
                    self.set_pulse(ch.pwm_channel, ch.current_value)?;
                    ch.changed = false;
                } 
                None => (),
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

fn render_ui(state: &AppConfig) -> Result<()> {
    use io::Write;
    use crossterm::{cursor::MoveTo, terminal::{Clear, ClearType}};
    let mut stdout = io::stdout();
    execute!(stdout, MoveTo(0, 0), Clear(ClearType::All))?;
    println!("*** ROBO CONTROLLER ***");
    println!("");
    println!("** Controls **");
    println!("Steering:");
    println!(" Left: A/←");
    println!(" Right: D/→");
    println!(" Forward: W/↑");
    println!(" Reverse/Brake: S/↓");
    println!(" Cut Throttle: Space");
    println!(" Full Throttle: T");
    println!(" Full Reverse: G");

    println!(" Quit: Q/ESC");
    println!("");
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
    arm_esc(&mut driver, &state)?;
    enable_raw_mode().context("Failed to enable raw terminal mode")?;
    execute!(io::stdout(), cursor::Hide)?;
    render_ui(&state)?;
    let result = run_loop(&mut driver, &mut state);
    let _ = disable_raw_mode();
    let _ = execute!(io::stdout(), cursor::Show);
    driver.safe_stop(&state)?;
    println!("\nRC robo controller stopped. Goodbye!");
    result
}

fn load_configuration() -> Result<AppConfig> {
    let cfg = Config::builder()
        .add_source(config::File::with_name("config"))
        .add_source(config::Environment::with_prefix("ROBOCONTROL"))
        .build()    
            .map_err(|e| anyhow::anyhow!("config error: {:?}", e));

    let raw: RawConfig = cfg.expect("whatever").try_deserialize()?;
    return AppConfig::from_raw(raw);
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

fn mavlink_to_pwm(input: u16, channel_config: ChannelConfig) -> u16 {
    if input == 0 {
        return channel_config.neutral;
    } else if input > 0 {
        return  input / 1000.0 * (channel_config.max - channel_config.neutral) + channel_config.neutral
    } else {
        return abs(input)/1000.0 * (channel_config.neutral - channel_config.min) + channel_config.min
    }
    return channel_config.neutral;
}

fn translate_message(msg :MavMessage, state: &AppConfig) -> Vec<AbsoluteControlOutput> {
    let mut outputs: Vec<AbsoluteControlOutput> = Vec::new();
    match msg {
        MavMessage::MANUAL_CONTROL(data) => {
            let throttle = mavlink_to_pwm(data.x, state.channel[state.controls.throttle]);
            outputs.push(AbsoluteControlOutput{
                channel: channel_from_u8(state.controls.throttle),
                value: throttle,
            });

            // Steering can be roll or yaw
            let steering = if data.y != 0 {
                mavlink_to_pwm(data.y, state.channel[state.controls.steering])
            } else {
                mavlink_to_pwm(data.r, state.channel[state.controls.steering])
            };

            outputs.push(AbsoluteControlOutput{
                channel: channel_from_u8(state.controls.steering),
                value: steering,
            }) 
        }
 
        MavMessage::RC_CHANNELS_OVERRIDE(data) => {
            // 18 available channels.
            let channels: [u16; 18] = [
                data.chan1_raw, data.chan2_raw, data.chan3_raw, data.chan4_raw,
                data.chan5_raw, data.chan6_raw, data.chan7_raw, data.chan8_raw,
                data.chan9_raw,  data.chan10_raw, data.chan11_raw, data.chan12_raw,
                data.chan13_raw, data.chan14_raw, data.chan15_raw, data.chan16_raw,
                data.chan17_raw, data.chan18_raw,
            ];

            for i in cmp::min(NUMBER_OF_CHANNELS, channels.len()) {
                outputs.push(AbsoluteControlOutput{
                    channel: channel_from_u8(i),
                    value: mavlink_to_pwm(channels[i], state.channel[i]),
                });
            }
        }
 
        MavMessage::SET_ACTUATOR_CONTROL_TARGET(data) => {
            // Only handle group 0 (primary flight control group).
            if data.group_mlx == 0 {
                outputs.push(AbsoluteControlOutput{
                    channel: channel_from_u8(state.controls.throttle),
                    value: mavlink_to_pwm(data.controls[3] as f32 * 1000, state.channel[state.controls.throttle]),
                });
                let throttle = (data.controls[3] as f32).clamp(-1.0, 1.0);

                outputs.push(AbsoluteControlOutput{
                    channel: channel_from_u8(state.controls.steering),
                    value: mavlink_to_pwm(data.controls[0] as f32 * 1000, state.channel[state.controls.steering]),
                });                
            }
        }
 
        // Ignore other messages
        _ => {},
    }

    return outputs;
}

fn run_loop(driver: &mut PwmDriver, state: &mut AppConfig, mavLinkService: MavLinkService) -> Result<()> {
    let mut time_since_last_heartbeat = Instant::now();
    loop {
        if time_since_last_heartbeat.elapsed() > HEARTBEAT_DURATION {
            mavLinkService.send_heart_beat()?;
            time_since_last_heartbeat = Instant::now();
        }

        match mavLinkService.connection.recv() {
            Ok((_header, msg)) => {
                let commands: Vec<AbsoluteControlOutput> = translate_message(msg, state); 

                // TODO

                driver.apply(state)?;
                render_ui(state)?;
            }
            Err(mavlink::error::MessageReadError::Io(e)) => {
                if e.kind() == std::io::ErrorKind::WouldBlock {
                    // no messages currently available to receive -- wait a bit
                    thread::sleep(Duration::from_millis(50));
                    continue;
                } else {
                    println!("recv error: {e:?}");
                    break;
                }
            }
            // messages that didn't get through due to parser errors are ignored
            _ => {}
        }
    }

    Ok(())
}
