//! Configuration loading and validation for RoboControl.
//!

use anyhow::{bail, Result};
use serde::Deserialize;
use std::collections::{HashMap, HashSet};

#[derive(Debug, Clone)]
pub struct ConfigStatic {
    pub i2c: I2cConfig,
    pub pwm: PwmConfig,
    pub mav: MavlinkConfig,
    pub controls: Controls,
}

#[derive(Debug, Clone, Deserialize)]
pub struct I2cConfig {
    pub address: u8,
    pub path: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct PwmConfig {
    pub prescale: u8,
}

#[derive(Debug, Clone, Deserialize)]
pub struct MavlinkConfig {
    pub port: u16,
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

#[derive(Debug)]
pub struct AppConfig {
    pub static_config: ConfigStatic,
    pub channel_blocks: [Option<ChannelConfigBlock>; 16],
    pub mavlink_to_index: HashMap<u8, usize>,
}

impl AppConfig {
    /// Returns the total number of configured (non-None) channel entries.
    pub fn channel_count(&self) -> usize {
        self.channel_blocks.iter().filter(|c| c.is_some()).count()
    }
}

/// Optional control-channel designations (0–15 indices into `[[channel]]`).
///
/// Fields are `Option<i64>` with `#[serde(default)]` so a missing `[controls]`
/// table (or missing key) deserializes to `None` rather than a hard error.
/// `i64` is used because the `config` crate surfaces integers as `i64` and a
/// `u8` default of `0` would be indistinguishable from a real channel-0 value.
#[derive(Debug, Default, Clone, Deserialize)]
struct Controls {
    #[serde(default)]
    steering: Option<i64>,
    #[serde(default)]
    throttle: Option<i64>,
}

#[derive(Debug, Deserialize)]
struct RawConfig {
    i2c: I2cConfig,
    pwm: PwmConfig,
    mav: MavlinkConfig,
    channel: Vec<RawChannelConfig>,
    #[serde(default)]
    controls: Controls,
}

fn build(raw: RawConfig) -> Result<AppConfig> {
    let mut blocks: [Option<ChannelConfigBlock>; 16] = std::array::from_fn(|_| None);
    let mut seen_pwm: HashSet<u8> = HashSet::new();
    let mut mavlink_to_index: HashMap<u8, usize> = HashMap::new();

    for (i, raw_ch) in raw.channel.iter().enumerate() {
        let ch_id = raw_ch.pwm_channel;

        // Validate pulse-width bounds: neutral must sit inside [min, max].
        if raw_ch.min > raw_ch.neutral {
            bail!(
                "Channel {}: min ({}) > neutral ({}) — neutral must be >= min",
                ch_id,
                raw_ch.min,
                raw_ch.neutral,
            );
        }
        if raw_ch.neutral > raw_ch.max {
            bail!(
                "Channel {}: neutral ({}) > max ({}) — neutral must be <= max",
                ch_id,
                raw_ch.neutral,
                raw_ch.max,
            );
        }
        if raw_ch.min > raw_ch.max {
            bail!(
                "Channel {}: min ({}) > max ({}) — invalid configuration",
                ch_id,
                raw_ch.min,
                raw_ch.max,
            );
        }

        if !seen_pwm.insert(ch_id) {
            bail!(
                "Duplicate pwm_channel {} at index {}: each PWM port may be assigned once",
                ch_id,
                i,
            );
        }
        if let Some(existing) = mavlink_to_index.get(&raw_ch.mavlink_channel) {
            bail!(
                "Duplicate mavlink_channel {} on entry {}: also set at index {}",
                raw_ch.mavlink_channel,
                i,
                existing,
            );
        }

        blocks[ch_id as usize] = Some(ChannelConfigBlock {
            pwm_channel: ch_id,
            min: raw_ch.min,
            max: raw_ch.max,
            neutral: raw_ch.neutral,
            mavlink_channel: raw_ch.mavlink_channel,
            max_step: raw_ch.max_step,
        });
        mavlink_to_index.insert(raw_ch.mavlink_channel, i);
    }

    Ok(AppConfig {
        static_config: ConfigStatic {
            i2c: raw.i2c,
            pwm: raw.pwm,
            mav: raw.mav,
            controls: raw.controls,
        },
        channel_blocks: blocks,
        mavlink_to_index,
    })
}

/// Load and validate configuration from `config.toml` + environment variables.
pub fn load() -> Result<AppConfig> {
    // `?` propagates `ConfigError` through anyhow so the full error chain
    // (cause + source) is preserved instead of being flattened into a string.
    let raw: RawConfig = config::Config::builder()
        .add_source(config::File::with_name("config"))
        .add_source(config::Environment::with_prefix("ROBOCONTROL"))
        .build()?
        .try_deserialize()?;

    let result = build(raw)?;

    log::info!(
        "Config loaded: {} active channels, I2C path={}, port={}",
        result.channel_count(),
        result.static_config.i2c.path,
        result.static_config.mav.port,
    );

    Ok(result)
}
