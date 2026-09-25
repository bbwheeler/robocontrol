//! RoboControl - PCA9685 PWM output driven by MAVLink commands.
//!
//! Runs on a Raspberry Pi with a PCA9685 on I2C. Loads `config.toml`,
//! listens for MAVLink telemetry on the configured UDP port, and maps
//! raw/scaled channel values to calibrated pulse widths with slew limiting.

mod config;
mod pwm;

use anyhow::{bail, Context, Result};
use config::AppConfig;
use linux_embedded_hal::I2cdev;
use mavlink::peek_reader::PeekReader;
use std::collections::HashMap;
use std::net::UdpSocket;
use std::time::{Duration, Instant};
use pwm::guard_pwm_value;
use pwm_pca9685::{Address, Channel, Pca9685};

const WATCHDOG_MS: u64 = 500;

fn main() -> Result<()> {
    env_logger::init();

    let app = config::load().context("failed to load config")?;

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
    let mut active_outputs: HashMap<u8, pwm::AbsoluteControlOutput> = HashMap::new();
    for ch_block in app.channel_blocks.iter().flatten() {
        let channel = match to_pca_channel(ch_block.pwm_channel) {
            Some(ch) => ch,
            None => bail!("pwm_channel {} is out of range (valid 0..=15)", ch_block.pwm_channel),
        };
        active_outputs.insert(
            ch_block.pwm_channel,
            pwm::AbsoluteControlOutput {
                channel,
                value: ch_block.neutral,
            },
        );
    }
    apply_all(&mut pwm_dev, &app, &active_outputs).context("apply startup neutral outputs")?;

    let mut last_message_time = Instant::now();
    let mut failsafe = false;

    loop {
        let now = Instant::now();

        // Read MAVLink message with a short timeout.
        udp.set_read_timeout(Some(Duration::from_millis(50))).ok();
        let (header, msg) = match recv_from(&udp, &mut [0u8; 4096]) {
            Some(m) => m,
            None if now.duration_since(last_message_time) > Duration::from_millis(WATCHDOG_MS) => {
                // Latched failsafe: only drive the neutral pulse once when the
                // watchdog first engages. While latched, skip the redundant I2C
                // write on each subsequent loop iteration. `last_message_time` is
                // deliberately NOT reset, so the latch holds until a real MAVLink
                // message arrives (which re-arms the system below).
                if !failsafe {
                    log::warn!("Watchdog timeout ({}ms) going neutral", WATCHDOG_MS);
                    send_neutral(&mut pwm_dev, &app, &active_outputs)?;
                    failsafe = true;
                }
                continue;
            }
            None => continue,
        };

        last_message_time = now;
        failsafe = false;

        match msg {
            mavlink::common::MavMessage::PARAM_VALUE(_) | mavlink::common::MavMessage::HEARTBEAT(_) | mavlink::common::MavMessage::STATUSTEXT(_) => {} // Ignore param values and common telemetry.
            mavlink::common::MavMessage::RC_CHANNELS_OVERRIDE(msg) => {
                let raw_values: Vec<u16> = vec![
                    msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                    msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
                ];
                log::debug!(
                    "RC_CHANNELS_OVERRIDE from sys#{} comp#{} (target_sys#{}): {:?}",
                    header.system_id, header.component_id, msg.target_system, raw_values,
                );
                process_raw_channels(&mut pwm_dev, &app, &mut active_outputs, &raw_values)?;
            }
            mavlink::common::MavMessage::RC_CHANNELS_RAW(msg) => {
                let raw_values: Vec<u16> = vec![
                    msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                    msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
                ];
                log::debug!(
                    "RC_CHANNELS_RAW from sys#{} comp#{}: {:?}",
                    header.system_id, header.component_id, raw_values,
                );
                process_raw_channels(&mut pwm_dev, &app, &mut active_outputs, &raw_values)?;
            }
            mavlink::common::MavMessage::SERVO_OUTPUT_RAW(msg) => {
                let raw_values: Vec<u16> = vec![
                    msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                    msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
                ];
                log::debug!(
                    "SERVO_OUTPUT_RAW from sys#{} comp#{}: {:?}",
                    header.system_id, header.component_id, raw_values,
                );
                process_raw_channels(&mut pwm_dev, &app, &mut active_outputs, &raw_values)?;
            }
            _ => {
                log::debug!("Ignoring MAVLink message: {:?}", msg);
            }
        }
    }
}

/// Translate 8 raw MAVLink channel values into slewed PWM duty counts and write
/// them to the PCA9685.
///
/// Shared by every RC-bearing message type (`RC_CHANNELS_OVERRIDE`,
/// `RC_CHANNELS_RAW`, `SERVO_OUTPUT_RAW`) since they all carry the same 8
/// raw pulse-width channels. `raw_values` is indexed by MAVLink channel
/// position (0-based), matching the order the sending GCS emitted.
fn process_raw_channels(
    pwm_dev: &mut PwmDriver,
    app: &AppConfig,
    active_outputs: &mut HashMap<u8, pwm::AbsoluteControlOutput>,
    raw_values: &[u16],
) -> Result<()> {
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

        let (prev_channel, new_value) = match active_outputs.get(&ch_block.pwm_channel) {
            Some(prev) if prev.value == duty => (Some(prev.channel.clone()), duty),
            Some(prev) => (Some(prev.channel.clone()), pwm::slew(prev.value, duty, ch_block.max_step)),
            None => (None, duty),
        };

        // Reuse the `Channel` cached on the previous `AbsoluteControlOutput` (populated at
        // startup). `to_pca_channel` is only the defensive fallback for a
        // channel that was never pre-populated (unreachable in practice).
        let channel = match prev_channel {
            Some(ch) => ch,
            None => to_pca_channel(ch_block.pwm_channel)
                .with_context(|| format!("pwm_channel {} is out of range (valid 0..=15)", ch_block.pwm_channel))?,
        };
        active_outputs.insert(
            ch_block.pwm_channel,
            pwm::AbsoluteControlOutput {
                channel,
                value: new_value,
            },
        );
    }

    apply_all(pwm_dev, app, active_outputs).context("apply PWM outputs")
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
            .with_context(|| "PCA9685 new".to_string())?;
        Ok(Self { pca })
    }

    fn set_prescale(&mut self, prescale: u8) -> Result<()> {
        self.pca
            .set_prescale(prescale)
            .context("PCA9685 set_prescale")
    }

    fn enable(&mut self) -> Result<()> {
        self.pca
            .enable()
            .context("PCA9685 enable")
    }

    fn set_channel_on_off(&mut self, ch: Channel, on: u16, off: u16) -> Result<()> {
        self.pca
            .set_channel_on_off(ch, on, off)
            .with_context(|| format!("PCA9685 set_channel_on_off on channel {:?}", ch))
    }
}

/// Open and initialize a PCA9685 device on the given I2C bus.
fn initialize_pca9685(path: &str, addr: u8) -> Result<PwmDriver> {
    let dev = PwmDriver::new(path, addr)?;
    Ok(dev)
}

/// Set the PCA9685 prescaler to achieve the desired PWM frequency.
fn set_prescale(pwm_dev: &mut PwmDriver, prescale: u8) -> Result<()> {
    pwm_dev.set_prescale(prescale)
}

/// Receive a MAVLink message from the UDP socket, returning None on timeout.
fn recv_from(socket: &UdpSocket, buf: &mut [u8; 4096]) -> Option<(mavlink::MavHeader, mavlink::common::MavMessage)> {
    match socket.recv_from(buf) {
        Ok((received, _addr)) => {
            if received > 0 {
                parse_mavlink(&buf[..received])
            } else {
                // Zero bytes received is equivalent to a timeout — avoid a hot spin.
                log::debug!("recv_from: 0 bytes received (timeout)");
                None
            }
        }
        Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => None,
        Err(e) => {
            log::error!("MAVLink socket read error: {}", e);
            None
        }
    }
}

/// Parse raw bytes as a MAVLink V2 message.
///
/// Returns the packet header (which carries the source system/component id)
/// alongside the decoded message, so callers can log who sent what.
fn parse_mavlink(data: &[u8]) -> Option<(mavlink::MavHeader, mavlink::common::MavMessage)> {
    let mut reader = PeekReader::new(data);
    match mavlink::read_v2_msg::<mavlink::common::MavMessage, _>(&mut reader) {
        Ok((header, msg)) => Some((header, msg)),
        Err(e) => {
            log::debug!("Parse error: {}", e);
            None
        }
    }
}

/// Write all active outputs to the PCA9685 hardware.
///
/// Every value is validated against its channel's calibrated `[min, max]`
/// bounds *before* it is written (code review item #16). An out-of-range
/// value is clamped into range and a warning is logged, instead of being
/// silently accepted by the chip's register clamping — which would hide the
/// underlying bug (e.g. a config typo with swapped min/max, or a computed
/// value that escaped the scaling helpers).
fn apply_all(pwm_dev: &mut PwmDriver, app: &AppConfig, outputs: &HashMap<u8, pwm::AbsoluteControlOutput>) -> Result<()> {
    for (channel, output) in outputs {
        let value = match app.channel_bounds(*channel) {
            Some((min, max)) => guard_pwm_value(*channel, output.value, min, max),
            // No bounds are known for this channel (shouldn't happen, as
            // outputs are only built from configured channels) — pass through.
            None => output.value,
        };
        pwm_dev.set_channel_on_off(output.channel, 0, value)?;
    }
    Ok(())
}

/// Send neutral to all channels (watchdog failsafe).
fn send_neutral(pwm_dev: &mut PwmDriver, app: &AppConfig, active_outputs: &HashMap<u8, pwm::AbsoluteControlOutput>) -> Result<()> {
    let mut neutral_outputs: HashMap<u8, pwm::AbsoluteControlOutput> = HashMap::new();
    for ch_block in app.channel_blocks.iter().flatten() {
        let new_value = match active_outputs.get(&ch_block.pwm_channel) {
            Some(prev) => pwm::slew(prev.value, ch_block.neutral, ch_block.max_step),
            None => ch_block.neutral,
        };
        let channel = to_pca_channel(ch_block.pwm_channel)
            .with_context(|| format!("pwm_channel {} is out of range (valid 0..=15)", ch_block.pwm_channel))?;
        neutral_outputs.insert(
            ch_block.pwm_channel,
            pwm::AbsoluteControlOutput {
                channel,
                value: new_value,
            },
        );
    }
    apply_all(pwm_dev, app, &neutral_outputs).context("apply watchdog PWM outputs")?;
    Ok(())
}

/// Convert the config's raw PWM channel number (0–15) into the pca9685 Channel enum.
///
/// Returns `None` for any value >= 16: the PCA9685 exposes only channels 0–15,
/// and callers must surface a clear error rather than silently mapping an
/// invalid channel onto C0. (Normally unreachable — `config::build` rejects
/// out-of-range `pwm_channel` values at config load time — but defense in depth.)
fn to_pca_channel(ch: u8) -> Option<Channel> {
    match ch {
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
        _ => None,
    }
}
