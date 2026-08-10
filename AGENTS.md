# AGENTS.md — RoboControl

## Architecture

`config.toml` drives I2C bus address, PWM prescale, and per-channel pulse-width bounds (min/max/neutral/mavlink_channel). The `ROBOCONTROL_` environment prefix also merges into config via the `config` crate.

## Developer commands

```bash
cargo build --release       # builds for Raspberry Pi target
RUST_LOG=debug cargo run    # run with logging
```

This runs on a Raspberry Pi with a PCA9685 on the I2C interface — it will not compile or execute on desktop.

## Gotchas

- **ESC arming**: On startup the program sends a 2-second neutral pulse (line 219). If the ESC needs custom calibration sequence, extend `arm_esc()`.
- **Watchdog failsafe**: After 500ms without a MAVLink message, all channels go neutral.
- **MAVLink ports**: Default listens on `udp in :14550` (set by `[mav].port` in config).
- **Channel mapping**: Each channel entry maps one PWM hardware channel to one MAVLink raw value (1000–2000 µs range) and/or scaled input (-10000–10000). Look at `translate_message()` and the `mavlink_*_to_pwm()` helpers for scaling logic.

## Beginning Tasks
Before you begin, follow these steps:
1. Make sure all existing changes have been checked in; if there are existing changes, commit them and push them to git.wheeli.ca.
2. Do a git fetch so that you have all of the latest changes.
3. Switch to a branch or create a branch appropriate for the changes that you will make

## Finishing Tasks
Once you complete any changes, additions, deletions, or modifications, follow these steps:
1. Check the code into a branch using git
2. Push the code to git.wheeli.ca
3. Open a Pull Request for the changes you just pushed
4. Add me (brian) as a reviewer on the Pull Request

## Credentials
Your credentials for git.wheeli.ca can be found in the parent directory (../credentials.md)