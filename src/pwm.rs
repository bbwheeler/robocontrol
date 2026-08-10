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

/// Scaled (-10000 – 10000) value meaning "use neutral" in `mavlink_scaled_to_pwm`.
pub const MAVLINK_SCALED_NEUTRAL: i16 = i16::MAX;

// ── Constants ──

/// Raw PWM range minimum (1000 µs).
const RAW_MIN: i32 = 1000;
/// Raw PWM range maximum (2000 µs).
const RAW_MAX: i32 = 2000;
/// Scaled PWM range min (-10000).
const SCALED_MIN: i32 = -10_000;
/// Scaled PWM range max (10000).
const SCALED_MAX: i32 = 10_000;

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

/// Convert a MAVLink raw pulse-width value (1000–2000 µs) to a calibrated PWM duty count.
///
/// Protocol mapping: `raw` values of 0 or `u16::MAX` are control-their-own special-cased
/// into the neutral position, matching ArduPilot convention for "unused channels."
pub fn mavlink_raw_to_pwm(input: u16, min: u16, max: u16, neutral: u16) -> u16 {
    if input == MAVLINK_CHANNEL_UNUSED_RAW || input == MAVLINK_CHANNEL_NEUTRAL_RAW {
        return neutral;
    }

    let clamped = input.clamp(RAW_MIN as u16, RAW_MAX as u16) as i32;

    // Map [1000..2000] → [-10000..10000] (scaled)
    let scaled: i32 = ((clamped - RAW_MIN) * SCALED_MAX / (RAW_MAX - RAW_MIN)) as i32;

    scaled_to_pwm(scaled, min, max, neutral)
}

/// Convert a MAVLink scaled control value (-10000 to 10000) to a calibrated PWM duty count.
///
/// Positive values map from `neutral` → `max`, negative from `neutral` → `min`.
pub fn scaled_to_pwm(input: i32, min: u16, max: u16, neutral: u16) -> u16 {

    let clamped = input.clamp(SCALED_MIN, SCALED_MAX);

    if clamped >= 0 {
        let range = (max - neutral) as i32;
        neutral + (clamped * range / SCALED_MAX) as u16
    } else {
        let range = (neutral - min) as i32;
        neutral.saturating_sub(((clamped.unsigned_abs() * range) / SCALED_MAX) as u16)
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
        // The special-cased values map to neutral via MAVLINK_SCALED_NEUTRAL
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
