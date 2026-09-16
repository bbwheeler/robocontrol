//! Pure mathematical functions for PWM scaling, slew rate limiting, and clamping.
//!
//! These functions are stateless and testable without any hardware dependency.

// ── MAVLink magic constants with documented protocol semantics ──

use pwm_pca9685::Channel;

/// Raw (1000–2000 microseconds) value meaning "this channel is unused."
pub const MAVLINK_CHANNEL_UNUSED_RAW: u16 = 0;

/// Raw (1000–2000 microseconds) value meaning "use neutral" when received via RC_CHANNELS_OVERRIDE.
/// This is a quirk of how ArduPilot/MAVProxy encode "pass through neutral."
pub const MAVLINK_CHANNEL_NEUTRAL_RAW: u16 = u16::MAX;

// ── Constants ──

/// The raw MAVLink pulse range this application assumes (1000–2000 µs).
///
/// This follows the RC_CHANNELS_OVERRIDE convention where a value of 0
/// means "unused" and `u16::MAX` means "pass through neutral". Different
/// MAVLink messages use different scaled ranges — these values are
/// application-specific assumptions, not protocol constants.
const APP_RAW_PULSE_MIN: i32 = 1000;
/// The raw MAVLink pulse range this application assumes (1000–2000 µs).
const APP_RAW_PULSE_MAX: i32 = 2000;

/// This application's scaled-range bound.
/// Maps the raw 1000–2000 µs range to `[-SCALED_BOUND, +SCALED_BOUND]`
/// to mimic the MAVLink `MANUAL_CONTROL` convention that ArduPilot
/// ground control stations commonly emit.
const SCALED_BOUND: i32 = 10_000;

/// Represents a per-channel PWM output value and its associated hardware channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AbsoluteControlOutput {
    pub channel: Channel,
    pub value: u16,
}

/// Slew-rate-limited PWM step: moves `current` toward `target` by at most `max_step`.
///
/// If `current == target`, returns `current` immediately (avoids unnecessary branch).
#[inline]
pub fn slew(current: u16, target: u16, max_step: u16) -> u16 {
    if current == target {
        return current;
    }
    if target > current {
        (current + max_step).min(target)
    } else {
        (current.saturating_sub(max_step)).max(target)
    }
}

/// Clamp a PWM value into the channel's `[cfg.min, cfg.max]` range.
pub fn clamp_to_channel(value: i32, min: u16, max: u16) -> u16 {
    (value.clamp(min as i32, max as i32)) as u16
}

/// Apply-time guard (code review item #16): keep a PWM value within a
/// channel's `[min, max]` bounds before it is written to the PCA9685.
///
/// A value already inside `[min, max]` is returned unchanged. An
/// out-of-range value is clamped into `[min, max]` and a warning is logged
/// that names the channel, the offending value, *and both* the min and max
/// bounds. Without this a config typo (e.g. swapped min/max) or an
/// out-of-range computed value would reach the hardware unclamped, and the
/// chip would silently clamp it and mask the underlying bug.
///
/// The function is total — it never panics. The normal path clamps within
/// `[min, max]`; the degenerate case where the bounds are inverted
/// (`min > max`, a configuration bug already rejected at load time) returns
/// the more conservative (smaller) of the two bounds instead of panicking.
pub fn guard_pwm_value(channel: u8, value: u16, min: u16, max: u16) -> u16 {
    if value < min || value > max {
        let clamped = if min <= max {
            value.clamp(min, max)
        } else {
            // Defensive only: bounds are inverted (rejected at config load
            // time). Fall back to the smaller bound rather than panicking.
            min.min(max)
        };
        log::warn!(
            "CH{}: PWM value {} out of configured range [{}..{}]; clamping to {}",
            channel, value, min, max, clamped
        );
        clamped
    } else {
        value
    }
}

/// Convert a MAVLink raw pulse-width value (1000–2000 µs) to a calibrated PWM duty count.
///
/// Protocol mapping: `raw` values of 0 or `u16::MAX` are control-their-own special-cased
/// into the neutral position, matching ArduPilot convention for "unused channels."
pub fn mavlink_raw_to_pwm(input: u16, min: u16, max: u16, neutral: u16) -> u16 {
    if input == MAVLINK_CHANNEL_UNUSED_RAW || input == MAVLINK_CHANNEL_NEUTRAL_RAW {
        return neutral;
    }

    let clamped = input.clamp(APP_RAW_PULSE_MIN as u16, APP_RAW_PULSE_MAX as u16) as i32;

    // Map [1000..2000] → [-SCALED_BOUND..SCALED_BOUND] (scaled)
    let scaled: i32 =
        (clamped - APP_RAW_PULSE_MIN) * SCALED_BOUND / (APP_RAW_PULSE_MAX - APP_RAW_PULSE_MIN);

    scaled_to_pwm(scaled, min, max, neutral)
}

/// Convert a MAVLink scaled control value (-10000 to 10000) to a calibrated PWM duty count.
///
/// Positive values map from `neutral` → `max`, negative from `neutral` → `min`.
pub fn scaled_to_pwm(input: i32, min: u16, max: u16, neutral: u16) -> u16 {
    let clamped = input.clamp(-SCALED_BOUND, SCALED_BOUND);

    if clamped >= 0 {
        let range = (max - neutral) as i32;
        neutral + (clamped * range / SCALED_BOUND) as u16
    } else {
        // Negative branch: `clamped` is in `[-SCALED_BOUND, 0)`, so
        // `clamped.unsigned_abs()` is `u32`. To keep the multiply and divide
        // on a single unsigned type (the 0–4095 PCA range fits in `u32`),
        // `range` and the bound are converted to `u32` here. `range =
        // neutral - min >= 0` (config guarantees `neutral >= min`), so the
        // conversion is lossless.
        let range: u32 = (neutral - min) as u32;
        let clamped_abs: u32 = clamped.unsigned_abs();
        neutral.saturating_sub(((clamped_abs * range) / SCALED_BOUND as u32) as u16)
    }
}

// ── Unit Tests ──

#[cfg(test)]
mod tests {
    use super::*;

    // ── slew tests ──

    #[test]
    fn slew_current_equals_target_returns_unchanged() {
        assert_eq!(slew(500, 500, 25), 500);
    }

    #[test]
    fn slew_up_steps_by_max_step() {
        assert_eq!(slew(100, 300, 25), 125);
    }

    #[test]
    fn slew_down_steps_by_max_step() {
        assert_eq!(slew(300, 100, 25), 275);
    }

    #[test]
    fn slew_up_exceeding_target_stops_at_target() {
        assert_eq!(slew(490, 510, 25), 510);
    }

    #[test]
    fn slew_down_exceeding_target_stops_at_target() {
        assert_eq!(slew(510, 490, 25), 490);
    }

    #[test]
    fn slew_zero_max_step_returns_target() {
        // max_step = 0: no intermediate step possible, so current stays (or target if equal)
        assert_eq!(slew(100, 300, 0), 100);
        assert_eq!(slew(300, 100, 0), 300);
        assert_eq!(slew(500, 500, 0), 500);
    }

    #[test]
    fn slew_underflow_saturates_at_target() {
        // current < max_step: (current + max_step) = 110, min(110, target=300) = 110
        let result = slew(10, 300, 100);
        assert_eq!(result, 110);
    }

    // ── clamp_to_channel tests ──

    #[test]
    fn clamp_above_max_returns_max() {
        assert_eq!(clamp_to_channel(700, 200, 600), 600);
        assert_eq!(clamp_to_channel(i32::MAX, 200, 600), 600);
    }

    #[test]
    fn clamp_below_min_returns_min() {
        assert_eq!(clamp_to_channel(100, 200, 600), 200);
        assert_eq!(clamp_to_channel(i32::MIN, 200, 600), 200);
    }

    #[test]
    fn clamp_in_range_returns_input() {
        assert_eq!(clamp_to_channel(400, 200, 600), 400);
    }

    // ── guard_pwm_value (item #16) tests ──
    //
    // `LogCapture` is a self-contained `log::Log` impl that records the
    // (level, message) pairs emitted through the `log` facade, so a test can
    // assert on the warning text without pulling in a new dev-dependency.
    //
    // `log::set_logger` is process-global and can only be installed once; the
    // four tests below share a single capture buffer, and each clears it in
    // `begin()` before asserting. `cargo test` runs tests on parallel threads,
    // so a `Mutex` serializes them to prevent one test's clear from wiping
    // another's just-captured records.

    use std::sync::Mutex;

    static LOG_BUF: std::sync::OnceLock<Mutex<Vec<(log::Level, String)>>> =
        std::sync::OnceLock::new();
    static LOGGER: std::sync::OnceLock<LogCapture> = std::sync::OnceLock::new();

    // Serializes the four guard tests so they cannot interleave clearing/
    // reading the shared log buffer (cargo test runs tests on parallel
    // threads). Held for the whole body of each test.
    static LOG_TEST_LOCK: std::sync::OnceLock<Mutex<()>> = std::sync::OnceLock::new();

    fn test_lock() -> std::sync::MutexGuard<'static, ()> {
        LOG_TEST_LOCK
            .get_or_init(|| Mutex::new(()))
            .lock()
            .unwrap()
    }

    struct LogCapture;

    impl LogCapture {
        fn buf() -> &'static Mutex<Vec<(log::Level, String)>> {
            LOG_BUF.get_or_init(|| Mutex::new(Vec::new()))
        }

        /// Install the capture logger exactly once and clear the buffer.
        fn begin() {
            let instance = LOGGER.get_or_init(|| LogCapture);
            let _ = log::set_logger(instance).map(|_| {
                log::set_max_level(log::LevelFilter::Warn);
            });
            Self::buf().lock().unwrap().clear();
        }

        fn warn_messages() -> Vec<String> {
            Self::buf()
                .lock()
                .unwrap()
                .iter()
                .filter(|(lvl, _)| *lvl == log::Level::Warn)
                .map(|(_, msg)| msg.clone())
                .collect()
        }
    }

    impl log::Log for LogCapture {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            // `log` gates records through `enabled`; accept them so the test
            // captures every emitted WARN record.
            true
        }

        fn log(&self, record: &log::Record) {
            Self::buf()
                .lock()
                .unwrap()
                .push((record.level(), record.args().to_string()));
        }

        fn flush(&self) {}
    }

    #[test]
    fn guard_in_range_returns_value_unchanged_and_logs_nothing() {
        let _guard = test_lock();
        LogCapture::begin();

        // Exactly on each bound and strictly inside.
        assert_eq!(guard_pwm_value(0, 250, 250, 550), 250);
        assert_eq!(guard_pwm_value(0, 550, 250, 550), 550);
        assert_eq!(guard_pwm_value(0, 400, 250, 550), 400);
        assert_eq!(guard_pwm_value(1, 300, 200, 600), 300);

        assert!(
            LogCapture::warn_messages().is_empty(),
            "in-range values must not log a warning, got: {:?}",
            LogCapture::warn_messages()
        );
    }

    #[test]
    fn guard_below_min_clamps_to_min_and_warns() {
        let _guard = test_lock();
        LogCapture::begin();

        let result = guard_pwm_value(0, 100, 250, 550);
        assert_eq!(result, 250);

        let warnings = LogCapture::warn_messages();
        assert_eq!(warnings.len(), 1);
        let msg = &warnings[0];
        assert!(msg.contains("CH0"), "warning should name the channel; got {msg}");
        assert!(msg.contains("100"), "warning should name the offending value; got {msg}");
        assert!(
            msg.contains("250") && msg.contains("550"),
            "warning should name BOTH min (250) and max (550) bounds; got {msg}"
        );
        assert!(msg.contains("clamp"), "warning should describe the fallback; got {msg}");
    }

    #[test]
    fn guard_above_max_clamps_to_max_and_warns() {
        let _guard = test_lock();
        LogCapture::begin();

        let result = guard_pwm_value(3, 900, 250, 550);
        assert_eq!(result, 550);

        let warnings = LogCapture::warn_messages();
        assert_eq!(warnings.len(), 1);
        let msg = &warnings[0];
        assert!(msg.contains("CH3"), "warning should name the channel; got {msg}");
        assert!(msg.contains("900"), "warning should name the offending value; got {msg}");
        assert!(
            msg.contains("250") && msg.contains("550"),
            "warning should name BOTH min (250) and max (550) bounds; got {msg}"
        );
    }

    #[test]
    fn guard_inverted_bounds_is_total_and_chooses_conservative_bound() {
        let _guard = test_lock();
        LogCapture::begin();

        // Item #16 explicitly calls out a config typo that swaps min/max.
        // The original two-branch guard would pass every value through
        // silently in this case; this implementation must handle it.
        // value = 300, bounds inverted min=550, max=250.
        // The "lower" bound (250) is the more conservative (smallest) safe
        // value to apply, so the guard returns it and warns.
        let result = guard_pwm_value(1, 300, 550, 250);
        assert_eq!(result, 250);

        let warnings = LogCapture::warn_messages();
        assert_eq!(warnings.len(), 1);
        let msg = &warnings[0];
        assert!(msg.contains("CH1"), "warning should name the channel; got {msg}");
        assert!(
            msg.contains("out of configured range"),
            "warning should describe the guard's fallback; got {msg}"
        );
        assert!(
            msg.contains("550") && msg.contains("250"),
            "warning should name BOTH bounds of the config typo (min=550, max=250); got {msg}"
        );

        // The guard is total and never panics even with inverted bounds — it
        // falls back to the smaller of the two bounds (250 here).
        assert_eq!(guard_pwm_value(1, 900, 550, 250), 250);
        assert_eq!(guard_pwm_value(1, 100, 550, 250), 250);
    }

    // ── scaled_to_pwm tests ──

    #[test]
    fn scaled_zero_maps_to_neutral_center_point() {
        let result = scaled_to_pwm(0, 200, 600, 400);
        assert_eq!(result, 400);
    }

    #[test]
    fn scaled_max_maps_to_max() {
        let result = scaled_to_pwm(10_000, 200, 600, 400);
        assert_eq!(result, 600);
    }

    #[test]
    fn scaled_min_maps_to_min() {
        let result = scaled_to_pwm(-10_000, 200, 600, 400);
        assert_eq!(result, 200);
    }

    #[test]
    fn scaled_neutral_magic_returns_neutral() {
        let result = scaled_to_pwm(i32::MAX, 200, 600, 400);
        assert_eq!(result, 400);
    }

    #[test]
    fn scaled_halfway_maps_linearly() {
        // 5000 / 10000 = 0.5 → neutral + (0.5 * max_range) = 400 + 100 = 500
        let result = scaled_to_pwm(5_000, 200, 600, 400);
        assert_eq!(result, 500);
    }

    #[test]
    fn scaled_negative_halfway_maps_linearly() {
        let result = scaled_to_pwm(-5_000, 200, 600, 400);
        assert_eq!(result, 300);
    }

    // ── raw_to_pwm tests ──

    #[test]
    fn raw_1000_maps_to_min() {
        let result = mavlink_raw_to_pwm(1000, 200, 600, 400);
        assert_eq!(result, 200);
    }

    #[test]
    fn raw_2000_maps_to_max() {
        let result = mavlink_raw_to_pwm(2000, 200, 600, 400);
        assert_eq!(result, 600);
    }

    #[test]
    fn raw_1500_maps_to_neutral() {
        let result = mavlink_raw_to_pwm(1500, 200, 600, 400);
        assert_eq!(result, 400);
    }

    #[test]
    fn raw_undefined_returns_neutral_via_magic_constant() {
        // The special-cased raw values map to neutral via the MAVLINK raw sentinel constants
        let result = mavlink_raw_to_pwm(MAVLINK_CHANNEL_UNUSED_RAW, 200, 600, 400);
        assert_eq!(result, 400);

        let result = mavlink_raw_to_pwm(MAVLINK_CHANNEL_NEUTRAL_RAW, 200, 600, 400);
        assert_eq!(result, 400);
    }

    #[test]
    fn raw_out_of_range_gets_clamped() {
        // Below range → clamps to min (raw=1000 equivalent)
        let result = mavlink_raw_to_pwm(500, 200, 600, 400);
        assert_eq!(result, 200);

        // Above range → clamps to max (raw=2000 equivalent)
        let result = mavlink_raw_to_pwm(3000, 200, 600, 400);
        assert_eq!(result, 600);
    }

    // ── asymmetric range tests ──

    #[test]
    fn asymmetric_up_range() {
        let result = scaled_to_pwm(10_000, 200, 800, 400);
        assert_eq!(result, 800); // neutral + max_range = 400 + 400
    }

    #[test]
    fn asymmetric_down_range() {
        let result = scaled_to_pwm(-10_000, 50, 600, 400);
        assert_eq!(result, 50); // neutral - max_range = 400 - 350
    }

    #[test]
    fn asymmetric_halfway_up() {
        let result = scaled_to_pwm(5_000, 200, 800, 400);
        assert_eq!(result, 600); // neutral + (50% * max_range) = 400 + (50% * 400) = 600
    }

    #[test]
    fn asymmetric_halfway_down() {
        let result = scaled_to_pwm(-5_000, 50, 800, 400);
        assert_eq!(result, 225); // neutral - (50% * down_range) = 400 - (50% * 350) = 400 - 175
    }
}
