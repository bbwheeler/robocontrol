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
use config::Config;

const NUMBER_OF_CHANNELS: u8 = 16;

// const I2C_BUS: &str = "/dev/i2c-1";
// const PCA9685_ADDRESS: u8 = 0x40;

// const PRESCALE: u8 = 100;

// const STEERING_CHANNEL: Channel = Channel::C0;
// const THROTTLE_CHANNEL: Channel = Channel::C1;

// const STEER_LEFT_MAX: u16    = 380;
// const STEER_CENTER: u16  = 500;
// const STEER_RIGHT_MAX: u16   = 620;

// const THROTTLE_REVERSE_MAX: u16 = 200;
// const THROTTLE_NEUTRAL: u16 = 400;
// const THROTTLE_FORWARD_MAX: u16 = 600;
// const THROTTLE_STEP: u16  = 20;
// const STEERING_STEP: u16  = 20;

struct RoboState {
    channel_values: [u16; NUMBER_OF_CHANNELS],
    channel_max_values: [u16; NUMBER_OF_CHANNELS],
    channel_min_values: [u16; NUMBER_OF_CHANNELS],
    channel_neutral_values: [u16; NUMBER_OF_CHANNELS],
    channel_steps: [u16; NUMBER_OF_CHANNELS],
}

impl RoboState {
    fn new() -> Self {
        Self {
            channel_values: [0; NUMBER_OF_CHANNELS],
            channel_max_values: [0; NUMBER_OF_CHANNELS],
            channel_min_values: [0; NUMBER_OF_CHANNELS],
            channel_steps: [0; NUMBER_OF_CHANNELS],
            channel_neutral_values: [0; NUMBER_OF_CHANNELS],
        }
    }
    // fn increase_channel(&mut self, channel)
    fn apply_throttle_forward(&mut self) { self.channel_values[1] = (self.channel_values[1] + self.channel_steps[1]).min(self.channel_max_values[1]); }
    fn apply_throttle_reverse(&mut self) { self.channel_values[1] = (self.channel_values[1] - channel_steps[1]).max(self.channel_min_values[1]); }
    fn steer_left(&mut self)             { self.channel_values[0] = (self.channel_values[0] - channel_steps[0]).max(self.channel_min_values[0]); }
    fn steer_right(&mut self)            { self.channel_steps[0] = (self.channel_steps[0] + self.channel_steps[0]).min(self.channel_max_values[0]); }
}

struct PwmDriver { pca: Pca9685<I2cdev> }

impl PwmDriver {
    fn new(ic2_bus_path: &str, ic2_address: u8, prescale: u8) -> Result<Self> {
        let i2c = I2cdev::new(ic2_bus_path)
            .with_context(|| format!("Failed to open I2C bus '{ic2_bus_path}'"))?;
        let address = Address::from(ic2_address);
        let mut pca = Pca9685::new(i2c, address)
            .map_err(|e| anyhow::anyhow!("PCA9685 init error: {:?}", e))?;

        pca.set_prescale(prescale)
            .map_err(|e| anyhow::anyhow!("set_prescale error: {:?}", e))?;

        pca.enable()
            .map_err(|e| anyhow::anyhow!("enable error: {:?}", e))?;

        Ok(Self { pca })
    }

    fn set_pulse(&mut self, channel: Channel, pulse: u16) -> Result<()> {
        self.pca.set_channel_on_off(channel, 0, pulse)
            .map_err(|e| anyhow::anyhow!("set_channel_on_off error: {:?}", e))
    }

    fn apply(&mut self, state: &RoboState) -> Result<()> {
        for c in 0..NUMBER_OF_CHANNELS {
            self.set_pulse(c, state.channel_values[c])?;

        } 
        Ok(())
    }

    fn safe_stop(&mut self) {
        for c in 0..NUMBER_OF_CHANNELS {
            self.set_pulse(c, state.channel_neutral_values[c])?;
        } 
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
    for c in 0..NUMBER_OF_CHANNELS {
        println!("Channel {}: {}", c, state.channel_values[c]);
    } 
    stdout.flush()?;
    Ok(())
}

fn arm_esc(driver: &mut PwmDriver) -> Result<()> {
    println!("Arming ESC — sending neutral throttle for 2 s…");
    driver.set_pulse(THROTTLE_CHANNEL, THROTTLE_NEUTRAL)?;
    driver.set_pulse(STEERING_CHANNEL, STEER_CENTER)?;

    for c in 0..NUMBER_OF_CHANNELS {
        driver.set_pulse(c, state.channel_neutral_values[c])?;
    } 
    thread::sleep(Duration::from_secs(2));
    println!("ESCs armed.");
    thread::sleep(Duration::from_millis(500));
    Ok(())
}

fn main() -> Result<()> {
    env_logger::init();

    let cfg= load_configuration()?;


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

fn load_configuration() -> Result<HashMap<String,String>> {
    let settings = Config::builder()
        .add_source(config::File::with_name("config"))
        .add_source(config::Environment::with_prefix("APP"))
        .build()    
            .map_err(|e| anyhow::anyhow!("config error: {:?}", e));

        return settings    
            .try_deserialize::<HashMap<String, String>>();
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
