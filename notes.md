## Compilation problems:

Compile with the Raspberry Pi 3b architecture as the target

Edit config file:
```
[target.armv7-unknown-linux-gnueabihf]
linker = "arm-linux-gnueabihf-gcc"
```