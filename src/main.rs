use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Address, Channel, Pca9685};
use std::collections::HashMap;
use config::Config;

fn main() {

    let cfg = Config::builder()
        .add_source(config::File::with_name("config"))
        .add_source(config::Environment::with_prefix("CONTROLLER"))
        .build()
        .unwrap();

    let cfg_map = cfg.try_deserialize::<HashMap<String, String>>().unwrap();

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = Address::default();
    let mut pwm = Pca9685::new(dev, address).unwrap();

    // The servos need 60Hz
    pwm.set_prescale(100).unwrap();
    pwm.enable().unwrap();

    let _dev = pwm.destroy(); // Get the I2C device back
}
