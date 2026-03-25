# RC Car Controller

Keyboard-controlled RC car via a Raspberry Pi → PCA9685 → Steering Servo + ESC.

---

## Hardware Wiring

```
Raspberry Pi          PCA9685
─────────────         ───────────────────
3.3 V  (Pin 1)  ───►  VCC
GND    (Pin 6)  ───►  GND
SDA    (Pin 3)  ───►  SDA
SCL    (Pin 5)  ───►  SCL

PCA9685               RC Hardware
─────────────         ───────────────────
V+  / GND       ───►  External 5–6 V BEC / servo rail
Channel 0       ───►  Steering servo signal wire
Channel 1       ───►  ESC signal wire
```

> **Power note:** Never power servos/ESC from the Pi's 3.3 V or 5 V pins.
> Use the PCA9685's external V+ terminal fed by a proper BEC or the ESC's
> built-in BEC (typically 5–6 V, up to 3 A).

---

## Raspberry Pi Setup

### 1. Enable I2C

```bash
sudo raspi-config
# → Interface Options → I2C → Enable
```

### 2. Verify the PCA9685 is detected

```bash
sudo apt install -y i2c-tools
i2cdetect -y 1
# You should see 0x40 (or your configured address) in the grid
```

### 3. Install Rust

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```

### 4. Add your user to the i2c group (avoid sudo for builds)

```bash
sudo usermod -aG i2c $USER
# Log out and back in for the change to take effect
```

---

## Build & Run

```bash
# Clone / copy the project to the Pi, then:
cd rc_car_controller
cargo build --release
./target/release/rc_car_controller
```

Set log level with:
```bash
RUST_LOG=debug ./target/release/rc_car_controller
```

---

## Controls

| Key          | Action                        |
|--------------|-------------------------------|
| W / ↑        | Throttle forward              |
| S / ↓        | Throttle reverse              |
| A / ←        | Steer left                    |
| D / →        | Steer right                   |
| Space        | Brake (neutral throttle)      |
| R            | Re-centre steering            |
| + / =        | Increase throttle step size   |
| -            | Decrease throttle step size   |
| Q / Esc      | Quit (safe shutdown)          |

---

## Calibration

All pulse-width constants are at the top of `src/main.rs`:

```rust
const STEER_LEFT_US:       f32 = 1000.0;   // full left
const STEER_CENTER_US:     f32 = 1500.0;   // centre
const STEER_RIGHT_US:      f32 = 2000.0;   // full right

const THROTTLE_REVERSE_US: f32 = 1000.0;   // full reverse
const THROTTLE_NEUTRAL_US: f32 = 1500.0;   // neutral / brake
const THROTTLE_FORWARD_US: f32 = 2000.0;   // full forward
```

Adjust these to match your servo's physical travel and your ESC's calibrated
endpoints. A USB oscilloscope or servo tester makes this much easier.

### ESC Arming

Most hobby ESCs require a neutral-throttle signal for ~1–2 s on power-up
before they will accept throttle commands. The program sends this arming pulse
automatically before entering the control loop.

If your ESC needs a specific calibration procedure (e.g., full-throttle then
neutral on power-up), perform that **before** running this program, or extend
the `arm_esc()` function accordingly.

---

## I2C Address

The default PCA9685 I2C address is `0x40` (all `A0–A5` pins floating/GND).
If you've changed the address jumpers:

```rust
const PCA9685_ADDRESS: u8 = 0x41;  // e.g., A0 tied high
```

---

## Safety Notes

- **Always** test with the car's drive wheels off the ground first.
- Start with a small `THROTTLE_STEP_US` (e.g., 25 µs) and increase gradually.
- The program sends a neutral-throttle pulse on exit (via `Drop` on `PwmDriver`).
- If the program crashes, the ESC will see no signal and most will failsafe to neutral.
