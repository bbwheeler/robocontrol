# RoboControl Code Review

## Critical / Bug

### 1. Channel lookup uses wrong index in `main.rs` (line 126)

```rust
for (i, &raw_val) in raw_values.iter().enumerate() {
    if let Some(ch_block) = app.channel_blocks[i].as_ref() { ... }
}
```

`channel_blocks` is indexed by **pwm_channel** (hardware port 0–15), but `raw_values.iter().enumerate()` iterates over MAVLink channels in message order. These two index spaces are unrelated — this only works by coincidence when `mavlink_channel - 1 == pwm_channel`.

The config already builds `mavlink_to_index: HashMap<u8, usize>` to map MAVLink channel IDs to positions, but it's never used in the message handler. The loop should use `mavlink_to_index` and a proper lookup keyed by MAVLink channel number (1-based).

### 2. Watchdog resets itself on every timeout

```rust
None if now.duration_since(last_message_time).as_millis() > WATCHDOG_MS as u128 => {
    log::warn!("Watchdog timeout ({}ms) going neutral", WATCHDOG_MS);
    send_neutral(&mut pwm_dev, &app, &active_outputs)?;
    last_message_time = now;  // ← resets timer immediately after firing
    continue;
}
```

Resetting `last_message_time` right after sending neutral means the watchdog can fire at most once per 500ms. If messages never arrive, neutrals keep re-sending every loop. More importantly, if you intended a hard failsafe (disarm and exit), this design prevents that behavior. Consider either:
- Exiting cleanly after N consecutive timeouts
- Not resetting the timer until a valid message is received again

### 3. No validation of min ≤ neutral ≤ max in config

The `build()` function checks for duplicate channels but never validates that `neutral` actually falls between `min` and `max`. A misconfigured neutral outside the range would produce inverted or clamped outputs silently, potentially causing sudden full-throttle commands on startup (safety hazard with real ESCs).

---

## Dead Code / Cleanup

### 4. `send_to()` is never called (`main.rs` line 204)

Unused function — remove it.

### 5. Duplicate struct: `Output` in `main.rs` mirrors `AbsoluteControlOutput` in `pwm.rs`

Both have `channel: Channel` and `value: u16`. Use `AbsoluteControlOutput` everywhere or inline the concept entirely. The duplication is unnecessary.

### 6. Unnecessary wrapper functions in `main.rs`

- `initialize_pca9685()` (line 193) — creates a `PwmDriver` and immediately returns it. Just call `PwmDriver::new()` directly.
- `set_prescale()` (line 199) — one-line pass-through to `pwm_dev.set_prescale()`. Call the method inline.
- `PwmDriver` itself adds zero logic beyond passthrough. The crate's `Pca9685<I2cdev>` can be used directly with minor error-wrapper adjustments.

### 7. `ActiveChannel` (`config.rs`) is never consumed outside of config construction

The struct, its `changed: bool` flag, and the `from_block()` constructor are built during config initialization but never read by `main.rs`. Either remove them or wire them into the main loop for dirty-tracking to avoid writing PWM registers that haven't changed.

### 8. `ConfigStatic::controls` is stored but unused in `main.rs`

The `steering` and `throttle` fields are validated during config build, stashed into `AppConfig`, but never referenced during message translation. Either use them (e.g., as semantic labels for logging) or remove the validation gate.

### 9. `RAW_MIN` / `SCALED_MIN` etc. in `pwm.rs` aren't actually protocol constants

These are hard-coded assumptions about MAVLink ranges, but different frames use different conventions (`MANUAL_CONTROL` uses -100..100, `POSITION_TARGET_LOCAL_NED` uses meters). Name them as *this application's* scaling parameters rather than implying they're universal.

---

## Performance / Correctness

### 10. `to_pca_channel()` called in hot loop (`main.rs` line 143)

Every MAVLink message triggers a full re-creation of `Channel::Cn` variants for every active channel via `to_pca_channel(chr_block.pwm_channel)`. The PCA channel mapping never changes — cache `Channel` once during startup inside the `Output` structs.

### 11. `channel_blocks[i]` iterates all 16 slots even though only ~2 are configured

The loop at line 125 iterates indices 0..7 of a `[Option<...>; 16]`. With the current (buggy) lookup, most iterations hit `None` and skip. Using a flat `Vec<&ChannelConfigBlock>` or an iterator over active configs would be clearer and faster.

### 12. `UDP_TIMEOUT` of 1ms burns CPU in a tight spin-loop

The loop at line 82 reads with a 1ms timeout, then immediately loops again. This results in ~1000 system calls per second even when idle. Consider using a larger timeout (e.g., 50ms) and checking the watchdog inside that window, or use async I/O (`tokio`/`async-std`) to avoid busy-waiting entirely.

### 13. WatchDOG_MS cast `as u128` is wrong type (`main.rs` line 89)

`duration_since().as_millis()` returns `u128`, so the comparison works, but the constant is declared as `u64`. The `as u128` cast suggests a confusion. Either change the constant to `u128` or compare differently.

---

## Error Handling

### 14. Lossy error wrapping in config deserialization (`config.rs` line 160-162)

```rust
.map_err(|e| anyhow::anyhow!("config build error: {}", e))?
.try_deserialize()?;
```

The `try_deserialize()` error is passed through `.context()` equivalent, losing the original `config::ConfigError` chain. Use `?` directly on a wrapped result or use `.with_context()` to preserve the full backtrace.

### 15. PCA9685 errors wrap already-stringified Debug output

```rust
.map_err(|e| anyhow::anyhow!("PCA9685 set_prescale error: {:?}", e))
```

`anyhow!` with `{:?}` embeds a debug string, making the error unsearchable and non-chainable. Use `Context::with_context()` or `anyhow!(...)` without debug formatting.

### 16. No protection against sending out-of-range PWM values to hardware

If a config typo swaps min/max, or `scaled_to_pwm` produces a value outside the PCA9685 register range (0–4095), the chip silently clamps — which may look correct but masks bugs. A runtime assertion at `apply_all()` that all values are within `[100, 4000]` would catch misconfigurations early.

---

## API / Design

### 17. `MavMessage` match doesn't handle `SERVO_OUTPUT_RAW` or other RC message types

Only `RC_CHANNELS_OVERRIDE` is translated. Many ground control stations emit `RC_CHANNELS` (pass-through of onboard receiver) instead. The match wildcard silently drops these. Either log a warning for unhandled RC-bearing messages, or support additional message types.

### 18. No distinction between "disarmed" and "armed" ESC state

On boot, all channels get neutral pulses (line 78). If the system goes into watchdog failsafe, it sends *slewed* neutral — which may not be sufficient to disarm an already-armed ESC that requires a specific low-pulse sequence. Document the expected disarm behavior or add an explicit disarm routine.

### 19. `to_pca_channel()` fallback returns `Channel::C0` for invalid input

Line 272: `_ => Channel::C0` silently maps out-of-range channel numbers to channel 0 instead of producing a build-time or runtime error. Use `Option<Channel>` and propagate, or assert at config load time that all `pwm_channel` values are in range (already validated by the `[0..16)` array bounds, but the function itself is unsafe).

---

## Minor / Style

### 20. `use std::collections::HashMap` imported in `main.rs` but a `Vec` would suffice for sequential application

All outputs are written every iteration via `apply_all`. A sorted `Vec<Output>` with index-based lookup would be faster and avoid hash overhead on an embedded target.

### 21. Unreachable code comment in `pwm.rs` (line 21-22)

```rust
// Note: this constant is only needed if external callers want sentinel neutral values...
// The pwm module functions themselves don't use it—they rely on 0 meaning neutral by default.
```

`MAVLINK_SCALED_NEUTRAL` is public but unused internally. Either document its purpose or make it `#[doc(hidden)]` to signal it's for external consumers only.

### 22. Edition 2024 in `Cargo.toml` — verify toolchain compatibility

Rust edition 2024 requires a recent nightly/toolchain (1.85+). The Raspberry Pi deployment environment should be verified to have a compatible rustc. If targeting stable, consider `edition = "2021"` until the team upgrades.

### 23. Log levels inconsistent

Startup config is logged at `INFO` in both `config::load()` (line 166) and `main()` (line 32). One of these duplicates can be removed to avoid log duplication.
