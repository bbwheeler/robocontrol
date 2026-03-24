use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Address, Channel, Pca9685};
use config::Config;

use std::{io, time::Duration, collections::HashMap};

use crossterm::{
    cursor::position,
    event::{poll, read, DisableMouseCapture, EnableMouseCapture, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode},
};

const POLLING_MILLIS: u16 = 60;

const STEERING_CHANNEL: u8 = 0;  // PCA9685 channel for steering
const THROTTLE_CHANNEL: u8 = 1;  // PCA9685 channel for ESC

const STEERING_FULL_LEFT: u16 = 200;    // min PWM for servo
const STEERING_FULL_RIGHT: u16 = 500;   // max PWM for servo (right)
const STEERING_CENTER: u16 = 350;  // neutral position

const THROTTLE_STOP: u16 = 300;    // ESC stop signal
const THROTTLE_FULL_FORWARD: u16 = 400; // ESC forward
const THROTTLE_BACKWARD: u16 = 200;// ESC backward

fn main() {

    let cfg = Config::builder()
        .add_source(config::File::with_name("config"))
        .add_source(config::Environment::with_prefix("CONTROLLER"))
        .build()
        .unwrap();

    let cfg_map = cfg.try_deserialize::<HashMap<String, String>>().unwrap();


    // TODO: Use config for address
    // TODO: Use config for which servo does what

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = Address::default();
    let mut pwm = Pca9685::new(dev, address).unwrap();

    let dev = I2cdev::new("/dev/i2c-1").unwrap(); // Raspberry Pi I²C bus
    let mut pwm = Pca9685::new(dev);


    // The servos need 60Hz
    pwm.set_prescale(100).unwrap(); // TODO: Figure out real frequency
    pwm.set_pwm_frequency(50.0).unwrap(); // TODO: Figure out real frequency
    pwm.enable().unwrap();

    enable_raw_mode()?;

    let mut stdout = io::stdout();
    execute!(stdout)?;

    if let Err(e) = main_loop() {
        println!("Error: {e:?}\r");
    }

    execute!(stdout, DisableMouseCapture)?;

    disable_raw_mode()

    let _dev = pwm.destroy(); // Give the I2C device back
}

// TODO: Use config for keybindings

fn read_input() -> Option<KeyCode> {
    if event::poll(Duration::from_millis(POLLING_MILLIS)).unwrap() {
        if let Event::Key(key_event) = event::read().unwrap() {
            return Some(key_event.code);
        }
    }
    None
}

fn main_loop() -> io::Result<()> {

    let mut servo_pos = SERVO_CENTER;
    let mut motor_pwm = MOTOR_STOP;

    loop {
        if let Some(key) = read_input() {
            match key {
                KeyCode::Char('0') => {
                    pwm.set_channel_on_off(Channel::from(MOTOR_CHANNEL), 0, MOTOR_STOP).unwrap();
                    sleep(Duration::from_secs(2)); // Arm the ESC
                }
                KeyCode::Char('a') => {
                    servo_pos = servo_pos.saturating_sub(10); // turn left
                    if servo_pos < SERVO_LEFT { servo_pos = SERVO_LEFT; }
                }
                KeyCode::Char('d') => {
                    servo_pos = servo_pos.saturating_add(10); // turn right
                    if servo_pos > SERVO_RIGHT { servo_pos = SERVO_RIGHT; }
                }
                KeyCode::Char('w') => motor_pwm = MOTOR_FORWARD,   // forward
                KeyCode::Char('s') => motor_pwm = MOTOR_BACKWARD,  // backward
                KeyCode::Char(' ') => motor_pwm = MOTOR_STOP, // stop
                KeyCode::Esc => break,  // exit
                _ => {}
            }

            // Update PCA9685
            pwm.set_channel_on_off(Channel::from(SERVO_CHANNEL), 0, servo_pos).unwrap();
            pwm.set_channel_on_off(Channel::from(MOTOR_CHANNEL), 0, motor_pwm).unwrap();
        }

        sleep(Duration::from_millis(POLLING_MILLIS/3)); // small delay to reduce CPU usage
    }
}
