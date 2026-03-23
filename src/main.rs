use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Address, Channel, Pca9685};

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = Address::default();
    let mut pwm = Pca9685::new(dev, address).unwrap();

    // The servos need 60Hz
    pwm.set_prescale(100).unwrap();
    pwm.enable().unwrap();

    let _dev = pwm.destroy(); // Get the I2C device back
}
