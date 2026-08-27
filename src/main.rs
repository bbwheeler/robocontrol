//! RoboControl - PCA9685 PWM output driven by MAVLink commands.
//!
//! Runs on a Raspberry Pi with a PCA9685 on I2C. Loads `config.toml`,
//! listens for MAVLink telemetry on the configured UDP port, and maps
//! raw/scaled channel values to calibrated pulse widths with slew limiting.

mod config;
mod pwm;

use anyhow::{Context, Result};
use config::AppConfig;
use linux_embedded_hal::I2cdev;
use std::collections::HashMap;
use std::net::UdpSocket;
use std::time::Instant;
use pwm_pca9685::{Address, Channel, Pca9685};

const WATCHDOG_MS: u64 = 500;

/// Internal representation of a PWM channel output.
#[derive(Debug, Clone)]
struct Output {
    channel: Channel,
    pub value: u16,
}

fn main() -> Result<()> {
    env_logger::init();

    let app = config::load().context("failed to load config")?;

    log::info!(
        "Starting RoboControl: {} channels, I2C={}, port={}",
        app.channel_blocks.iter().filter(|c| c.is_some()).count(),
        app.static_config.i2c.path,
        app.static_config.mav.port,
    );

    // Open the MAVLink UDP socket for receiving commands.
    let bind_addr = format!("0.0.0.0:{}", app.static_config.mav.port);
    let udp = UdpSocket::bind(&bind_addr).with_context(|| format!("bind {}", bind_addr))?;
    log::info!("MAVLink listener on {}", bind_addr);

    // Initialize PCA9685 driver using linux-embedded-hal.
    let dev_path = &app.static_config.i2c.path;
    let addr = app.static_config.i2c.address;
    let mut pwm_dev = initialize_pca9685(dev_path, addr)
        .with_context(|| format!("init pca9685 on {}", dev_path))?;
    set_prescale(&mut pwm_dev, app.static_config.pwm.prescale)
        .context("set prescale")?;
    pwm_dev.enable().context("enable pca9685")?;
    log::info!("PCA9685 initialized on {}", dev_path);

    // Log configured channels.
    for ch_block in app.channel_blocks.iter().flatten() {
        log::debug!(
            "CH{}: min={} max={} neutral={} mavlink_ch={} step={}",
            ch_block.pwm_channel,
            ch_block.min,
            ch_block.max,
            ch_block.neutral,
            ch_block.mavlink_channel,
            ch_block.max_step,
        );
    }

    // Apply neutral pulses to all channels on startup (ESC arming).
    let mut active_outputs: HashMap<u8, Output> = HashMap::new();
    for ch_block in app.channel_blocks.iter().flatten() {
        active_outputs.insert(
            ch_block.pwm_channel,
            Output {
                channel: to_pca_channel(ch_block.pwm_channel),
                value: ch_block.neutral,
            },
        );
    }
    apply_all(&mut pwm_dev, &active_outputs).context("apply startup neutral outputs")?;

    let mut last_message_time = Instant::now();

    loop {
        let now = Instant::now();

        // Read MAVLink message with a short timeout.
        udp.set_read_timeout(Some(std::time::Duration::from_millis(1))).ok();
        let msg = match recv_from(&udp, &mut [0u8; 4096]) {
            Some(m) => m,
            None if now.duration_since(last_message_time).as_millis() > WATCHDOG_MS as u128 => {
                log::warn!("Watchdog timeout ({}ms) going neutral", WATCHDOG_MS);
                send_neutral(&mut pwm_dev, &app, &active_outputs)?;
                last_message_time = now;
                continue;
            }
            None => continue,
        };

        last_message_time = now;

        match msg {
            mavlink::common::MavMessage::ParamValue { .. } | mavlink::common::MavMessage::Heartbeat { .. } | mavlink::common::MavMessage::Statustext { .. } => {} // Ignore param values and common telemetry.
            mavlink::common::MavMessage::RC_CHANNELS_OVERRIDE {
                target_network_id: _,
                target_system_id,
                target_component_id: _,
                ref chan1_raw,
                ref chan2_raw,
                ref chan3_raw,
                ref chan4_raw,
                ref chan5_raw,
                ref chan6_raw,
                ref chan7_raw,
                ref chan8_raw,
            } => {
                let raw_values: Vec<u16> = vec![
                    *chan1_raw, *chan2_raw, *chan3_raw, *chan4_raw,
                    *chan5_raw, *chan6_raw, *chan7_raw, *chan8_raw,
                ];

                log::debug!(
                    "RC_CHANNELS_OVERRIDE from sys#{}: {:?}",
                    target_system_id, raw_values,
                );

                for ch_block in app.channel_blocks.iter().flatten() {
                    let mav_ch = (ch_block.mavlink_channel - 1) as usize;
                    if mav_ch >= raw_values.len() {
                        continue;
                    }
                    let raw_val = raw_values[mav_ch];

                    // Translate raw MAVLink pulse width to calibrated PWM duty count.
                    let duty = pwm::mavlink_raw_to_pwm(
                        raw_val, ch_block.min, ch_block.max, ch_block.neutral,
                    );

                    let new_value = match active_outputs.get(&ch_block.pwm_channel) {
                        Some(prev) if prev.value == duty => duty,
                        Some(prev) => pwm::slew(prev.value, duty, ch_block.max_step),
                        None => duty,
                    };

                    active_outputs.insert(
                        ch_block.pwm_channel,
                        Output {
                            channel: to_pca_channel(ch_block.pwm_channel),
                            value: new_value,
                        },
                    );
                }

                apply_all(&mut pwm_dev, &active_outputs)
                    .context("apply PWM outputs")?;
            }
            _ => {
                log::debug!("Ignoring MAVLink message: {:?}", msg);
            }
        }
    }
}

/// Wrapper around the PCA9685 hardware abstraction.
struct PwmDriver {
    pca: Pca9685<I2cdev>,
}

impl PwmDriver {
    fn new(path: &str, addr: u8) -> Result<Self> {
        let i2c = I2cdev::new(path)
            .with_context(|| format!("Failed to open I2C bus '{path}'"))?;
        let address = Address::from(addr);
        let pca = Pca9685::new(i2c, address)
            .with_context("PCA9685 new")?;
        Ok(Self { pca })
    }

    fn set_prescale(&mut self, prescale: u8) -> Result<()> {
        self.pca
            .set_prescale(prescale)
            .map_err(|e| anyhow::anyhow!("PCA9685 set_prescale error: {:?}", e))
    }

    fn enable(&mut self) -> Result<()> {
        self.pca
            .enable()
            .map_err(|e| anyhow::anyhow!("PCA9685 enable error: {:?}", e))
    }

    fn set_channel_on_off(&mut self, ch: Channel, on: u16, off: u16) -> Result<()> {
        self.pca
            .set_channel_on_off(ch, on, off)
            .map_err(|e| anyhow::anyhow!("PCA9685 set_channel_on_off error: {:?}", e))
    }
}

/// Open and initialize a PCA9685 device on the given I2C bus.
fn initialize_pca9685(path: &str, addr: u8) -> Result<PwmDriver> {
    let mut dev = PwmDriver::new(path, addr)?;
    Ok(dev)
}

/// Set the PCA9685 prescaler to achieve the desired PWM frequency.
fn set_prescale(pwm_dev: &mut PwmDriver, prescale: u8) -> Result<()> {
    pwm_dev.set_prescale(prescale)
}

/// Send a single MAVLink UDP packet containing given message bytes.
fn send_to<M: Into<Vec<u8>>>(socket: &UdpSocket, peer_addr: &str, data: M) -> Result<()> {
    socket.send_to(data.into().as_slice(), peer_addr)?;
    Ok(())
}

/// Receive a MAVLink message from the UDP socket, returning None on timeout.
fn recv_from(socket: &UdpSocket, buf: &mut [u8; 4096]) -> Option<mavlink::common::MavMessage> {
    match socket.recv_from(buf) {
        Ok((n, _)) if n > 0 => parse_mavlink(&buf[..n]),
        Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => None,
        Err(e) => {
            log::error!("MAVLink socket read error: {}", e);
            None
        }
    }
}

/// Parse raw bytes as a MAVLink message.
fn parse_mavlink(data: &[u8]) -> Option<mavlink::common::MavMessage> {
    match mavlink::from_bytes::<mavlink::common::MavMessage>(data) {
        Ok((msg, _)) => Some(msg),
        Err(e) => {
            log::debug!("Parse error: {}", e);
            None
        }
    }
}

/// Write all active outputs to the PCA9685 hardware.
fn apply_all(pwm_dev: &mut PwmDriver, outputs: &HashMap<u8, Output>) -> Result<()> {
    for output in outputs.values() {
        pwm_dev.set_channel_on_off(output.channel, 0, output.value)?;
    }
    Ok(())
}

/// Send neutral to all channels (watchdog failsafe).
fn send_neutral(pwm_dev: &mut PwmDriver, app: &AppConfig, active_outputs: &HashMap<u8, Output>) -> Result<()> {
    let mut neutral_outputs: HashMap<u8, Output> = HashMap::new();
    for ch_block in app.channel_blocks.iter().flatten() {
        let new_value = match active_outputs.get(&ch_block.pwm_channel) {
            Some(prev) => pwm::slew(prev.value, ch_block.neutral, ch_block.max_step),
            None => ch_block.neutral,
        };
        neutral_outputs.insert(
            ch_block.pwm_channel,
            Output {
                channel: to_pca_channel(ch_block.pwm_channel),
                value: new_value,
            },
        );
    }
    apply_all(pwm_dev, &neutral_outputs).context("apply watchdog PWM outputs")?;
    Ok(())
}

/// Convert the config's raw PWM channel number (0–15) into the pca9685 Channel enum.
fn to_pca_channel(ch: u8) -> Channel {
    match ch {
        0  => Channel::C0,  1  => Channel::C1,
        2  => Channel::C2,  3  => Channel::C3,
        4  => Channel::C4,  5  => Channel::C5,
        6  => Channel::C6,  7  => Channel::C7,
        8  => Channel::C8,  9  => Channel::C9,
        10 => Channel::C10, 11 => Channel::C11,
        12 => Channel::C12, 13 => Channel::C13,
        14 => Channel::C14, 15 => Channel::C15,
        _ => Channel::C0,
    }
}
