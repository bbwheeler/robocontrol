//! Configuration loading and validation for RoboControl.
//!

use serde::Deserialize;
use anyhow::bail;
use std::collections::{HashMap, HashSet};

#[derive(Debug, Clone)]
pub struct ConfigStatic {
    pub i2c: I2cConfig,
    pub pwm: PwmConfig,
    pub mav: MavlinkConfig,
    pub controls: Controls,
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

#[derive(Debug, Clone, Deserialize)]
pub struct RawChannelConfig {
    pub pwm_channel: u8,
    pub min: u16,
    pub max: u16,
    pub neutral: u16,
    pub mavlink_channel: u8,
    pub max_step: u16,
}

#[derive(Debug, Clone)]
pub struct ChannelConfigBlock {
    pub pwm_channel: u8,
    pub min: u16,
    pub max: u16,
    pub neutral: u16,
    pub mavlink_channel: u8,
    pub max_step: u16,
}

#[derive(Debug, Clone)]
pub struct ActiveChannel {
    pub pwm_channel: u8,
    pub current_value: u16,
    pub changed: bool,
}

impl ActiveChannel {
    fn from_block(block: &ChannelConfigBlock) -> Self {
        Self {
            pwm_channel: block.pwm_channel,
            current_value: block.neutral,
            changed: true,
        }
    }
}

#[derive(Debug)]
pub struct AppConfig {
    pub static_config: ConfigStatic,
    pub channels: [Option<ActiveChannel>; 16],
    pub channel_blocks: [Option<ChannelConfigBlock>; 16],
    pub mavlink_to_index: HashMap<u8, usize>,
}

#[derive(Debug, Clone, Deserialize)]
struct Controls {
    steering: u8,
    throttle: u8,
}

#[derive(Debug, Deserialize)]
struct RawConfig {
    i2c: I2cConfig,
    pwm: PwmConfig,
    mav: MavlinkConfig,
    channel: Vec<RawChannelConfig>,
    controls: Controls,
}

fn build(raw: RawConfig) -> Result<AppConfig> {
    let mut blocks: [Option<ChannelConfigBlock>; 16] = [None; 16];
    let mut seen_pwm: HashSet<u8> = HashSet::new();
    let mut mavlink_to_index: HashMap<u8, usize> = HashMap::new();

    for (i, raw_ch) in raw.channel.iter().enumerate() {
        let ch_id = raw_ch.pwm_channel;
        if !seen_pwm.insert(ch_id) {
            bail!("Duplicate pwm_channel {} at index {}: each PWM port may be assigned once", ch_id, i);
        }
        if let Some(existing) = mavlink_to_index.get(&raw_ch.mavlink_channel) {
            bail!("Duplicate mavlink_channel {} on entry {}: also set at index {}", raw_ch.mavlink_channel, i, existing);
        }

        blocks[raw_ch.pwm_channel as usize] = Some(ChannelConfigBlock {
            pwm_channel: ch_id,
            min: raw_ch.min,
            max: raw_ch.max,
            neutral: raw_ch.neutral,
            mavlink_channel: raw_ch.mavlink_channel,
            max_step: raw_ch.max_step,
        });
        mavlink_to_index.insert(raw_ch.mavlink_channel, i);
    }

    // Validate that control channel indices are valid.
    if raw.controls.steering >= 16 || raw.channel.get(raw.controls.steering as usize).is_none() {
        bail!("controls.steering={} out of range or no [[channel]] at that index", raw.controls.steering);
    }
    if raw.controls.throttle >= 16 || raw.channel.get(raw.controls.throttle as usize).is_none() {
        bail!("controls.throttle={} out of range or no [[channel]] at that index", raw.controls.throttle);
    }

    let channels: [Option<ActiveChannel>; 16] = blocks
        .iter()
        .map(|b| b.as_ref().map(ActiveChannel::from_block))
        .collect::<Vec<_>>()
        .try_into()
        .unwrap();

    Ok(AppConfig {
        channels,
        static_config: ConfigStatic {
            i2c: raw.i2c,
            pwm: raw.pwm,
            mav: raw.mav,
            controls: Controls {
                steering: raw.controls.steering,
                throttle: raw.controls.throttle,
            },
        },
        channel_blocks: blocks,
        mavlink_to_index,
    })
}

/// Load and validate configuration from `config.toml` + environment variables.
pub fn load() -> Result<AppConfig> {
    let raw: RawConfig = config::Config::builder()
        .add_source(config::File::with_name("config"))
        .add_source(config::Environment::with_prefix("ROBOCONTROL"))
        .build()
        .map_err(|e| anyhow::anyhow!("config build error: {}", e))?
        .try_deserialize()?;

    let result = build(raw)?;

    log::info!(
        "Config loaded: {} active channels, I2C path={}, port={}",
        result.channel_blocks.iter().filter(|c| c.is_some()).count(),
        result.static_config.i2c.path,
        result.static_config.mav.port,
    );

    Ok(result)
}
