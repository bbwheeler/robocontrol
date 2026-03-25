//! RoboControl
//! Control servos and ESCs with a PCA9685
use std::io;
use std::time::Duration;
use std::thread;

use anyhow::{Context, Result};
use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind},
    terminal::{disable_raw_mode, enable_raw_mode},
    execute,
    cursor,
};
use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Address, Channel, Pca9685};

const I2C_BUS: &str = "/dev/i2c-1";
const PCA9685_ADDRESS: u8 = 0x40;


const PRESCALE: u8 = 100;

const STEERING_CHANNEL: Channel = Channel::C0;
const THROTTLE_CHANNEL: Channel = Channel::C1;

const STEER_LEFT: u16    = 1024;
const STEER_CENTER: u16  = 2047;
const STEER_RIGHT: u16   = 3071;
const THROTTLE_REVERSE: u16 = 1024;
const THROTTLE_NEUTRAL: u16 = 2047;
const THROTTLE_FORWARD: u16 = 3071;
const THROTTLE_STEP: u16  = 25;
const STEERING_STEP: u16  = 50;

struct RoboState {
    throttle: u16,
    steering: u16,
    throttle_step: u16,
}

impl RoboState {
    fn new() -> Self {
        Self { throttle: THROTTLE_NEUTRAL, steering: STEER_CENTER, throttle_step: THROTTLE_STEP }
    }
    fn apply_throttle_forward(&mut self) { self.throttle = (self.throttle + self.throttle_step).min(THROTTLE_FORWARD); }
    fn apply_throttle_reverse(&mut self) { self.throttle = (self.throttle - self.throttle_step).max(THROTTLE_REVERSE); }
    fn apply_full_throttle(&mut self) { self.throttle = THROTTLE_FORWARD; }
    fn apply_full_throttle_reverse(&mut self) { self.throttle = THROTTLE_REVERSE; }
    fn apply_brake(&mut self)            { self.throttle = THROTTLE_NEUTRAL; }
    fn steer_left(&mut self)             { self.steering = (self.steering - STEERING_STEP).max(STEER_LEFT); }
    fn steer_right(&mut self)            { self.steering = (self.steering + STEERING_STEP).min(STEER_RIGHT); }
}

struct PwmDriver { pca: Pca9685<I2cdev> }

impl PwmDriver {
    fn new() -> Result<Self> {
        let i2c = I2cdev::new(I2C_BUS)
            .with_context(|| format!("Failed to open I2C bus '{I2C_BUS}'"))?;
        let address = Address::from(PCA9685_ADDRESS);
        let mut pca = Pca9685::new(i2c, address)
            .map_err(|e| anyhow::anyhow!("PCA9685 init error: {:?}", e))?;

        pca.set_prescale(PRESCALE)
            .map_err(|e| anyhow::anyhow!("set_prescale error: {:?}", e))?;

        pca.enable()
            .map_err(|e| anyhow::anyhow!("enable error: {:?}", e))?;

        thread::sleep(Duration::from_millis(10));
        Ok(Self { pca })
    }

    fn set_pulse(&mut self, channel: Channel, pulse: u16) -> Result<()> {
        self.pca.set_channel_on_off(channel, 0, pulse)
            .map_err(|e| anyhow::anyhow!("set_channel_on_off error: {:?}", e))
    }

    fn apply(&mut self, state: &RoboState) -> Result<()> {
        self.set_pulse(STEERING_CHANNEL, state.steering)?;
        self.set_pulse(THROTTLE_CHANNEL, state.throttle)?;
        Ok(())
    }

    fn safe_stop(&mut self) {
        let _ = self.set_pulse(THROTTLE_CHANNEL, THROTTLE_NEUTRAL);
        let _ = self.set_pulse(STEERING_CHANNEL, STEER_CENTER);
    }
}

impl Drop for PwmDriver {
    fn drop(&mut self) { self.safe_stop(); }
}

fn render_ui(state: &RoboState) -> Result<()> {
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
    println!("Throttle: {}", state.throttle);
    println!("Steering: {}", state.steering);
    println!("Throttle Step: {}", state.throttle_step);
    stdout.flush()?;
    Ok(())
}

fn arm_esc(driver: &mut PwmDriver) -> Result<()> {
    println!("Arming ESC — sending neutral throttle for 2 s…");
    driver.set_pulse(THROTTLE_CHANNEL, THROTTLE_NEUTRAL)?;
    driver.set_pulse(STEERING_CHANNEL, STEER_CENTER)?;
    thread::sleep(Duration::from_secs(2));
    println!("ESC armed. Starting controller…");
    thread::sleep(Duration::from_millis(500));
    Ok(())
}

fn main() -> Result<()> {
    env_logger::init();
    let mut driver = PwmDriver::new().context("Failed to initialise PCA9685")?;
    arm_esc(&mut driver)?;
    let mut state = RoboState::new();
    enable_raw_mode().context("Failed to enable raw terminal mode")?;
    execute!(io::stdout(), cursor::Hide)?;
    render_ui(&state)?;
    let result = run_loop(&mut driver, &mut state);
    let _ = disable_raw_mode();
    let _ = execute!(io::stdout(), cursor::Show);
    driver.safe_stop();
    println!("\nRC robo controller stopped. Goodbye!");
    result
}

fn run_loop(driver: &mut PwmDriver, state: &mut RoboState) -> Result<()> {
    loop {
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc   => break,
                        KeyCode::Char('w') | KeyCode::Char('W') | KeyCode::Up    => state.apply_throttle_forward(),
                        KeyCode::Char('s') | KeyCode::Char('S') | KeyCode::Down  => state.apply_throttle_reverse(),
                        KeyCode::Char('t') | KeyCode::Char('T')                  => state.apply_full_throttle(),
                        KeyCode::Char('g') | KeyCode::Char('G')                  => state.apply_full_throttle_reverse(),
                        KeyCode::Char(' ')                                       => state.apply_brake(),
                        KeyCode::Char('a') | KeyCode::Char('A') | KeyCode::Left  => state.steer_left(),
                        KeyCode::Char('d') | KeyCode::Char('D') | KeyCode::Right => state.steer_right(),
                        _                                                        => continue,
                    }
                    driver.apply(state)?;
                    render_ui(state)?;
                }
            }
        }
    }
    Ok(())
}
