VL53L0X time-of-flight ranging sensor
=====================================

[![Build Status](https://travis-ci.org/d2r2/go-vl53l0x.svg?branch=master)](https://travis-ci.org/d2r2/go-vl53l0x)
[![Go Report Card](https://goreportcard.com/badge/github.com/d2r2/go-vl53l0x)](https://goreportcard.com/report/github.com/d2r2/go-vl53l0x)
[![GoDoc](https://godoc.org/github.com/d2r2/go-vl53l0x?status.svg)](https://godoc.org/github.com/d2r2/go-vl53l0x)
[![MIT License](http://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)

VL53L0X ([general specification](https://raw.github.com/d2r2/go-vl53l0x/master/docs/vl53l0x.pdf), [native C code API specification](https://raw.github.com/d2r2/go-vl53l0x/master/docs/en.DM00279088.pdf)) is a world’s smallest time-of-flight ranging and gesture detection sensor from STMicroelectronics. Easily integrated with Arduino and Raspberry PI via i2c communication interface:
![image](https://raw.github.com/d2r2/go-vl53l0x/master/docs/cjvl53l0xv2.jpg)

Sensor functionality is based on laser diode emission with following photodetector signal registration. Average time duration between emission and registration is a "time-of-flight", which translates to range distance.

Here is a library written in [Go programming language](https://golang.org/) for Raspberry PI and counterparts, which gives you in the output measured range value (making all necessary i2c-bus interacting and values computing).

This library is an adaptation and translation of well-formed C++ code written by www.pololu.com to Golang, taken from https://github.com/pololu/vl53l0x-arduino.


Golang usage
------------

```go
func main() {
    // Create new connection to i2c-bus on 0 line with address 0x29.
    // Use i2cdetect utility to find device address over the i2c-bus
    i2c, err := i2c.NewI2C(0x29, 0)
    if err != nil {
        log.Fatal(err)
    }
    defer i2c.Close()

    sensor := vl53l0x.NewVl53l0x()
    // It's highly recommended to reset sensor each time before repeated initialization.
    err = sensor.Reset(i2c)
    if err != nil {
        log.Fatal(err)
    }
    // By default, sensor initialized with "RegularRange" and "RegularAccuracy" parameters.
    err = sensor.Init(i2c)
    if err != nil {
        log.Fatal(err)
    }
    rng, err := sensor.ReadRangeSingleMillimeters(i2c)
    if err != nil {
        log.Fatal(err)
    }
    log.Printf("Measured range = %v mm", rng)
```


Getting help
------------

GoDoc [documentation](http://godoc.org/github.com/d2r2/go-vl53l0x)


Installation
------------

```bash
$ go get -u github.com/d2r2/go-vl53l0x
```


Troubleshooting
---------------

- *How to obtain fresh Golang installation to RPi device (either any RPi clone):*
If your RaspberryPI golang installation taken by default from repository is outdated, you may consider
to install actual golang manually from official Golang [site](https://golang.org/dl/). Download
tar.gz file containing armv6l in the name. Follow installation instructions.

- *How to enable I2C bus on RPi device:*
If you employ RaspberryPI, use raspi-config utility to activate i2c-bus on the OS level.
Go to "Interfacing Options" menu, to active I2C bus.
Probably you will need to reboot to load i2c kernel module.
Finally you should have device like /dev/i2c-1 present in the system.

- *How to find I2C bus allocation and device address:*
Use i2cdetect utility in format "i2cdetect -y X", where X may vary from 0 to 5 or more,
to discover address occupied by peripheral device. To install utility you should run
`apt install i2c-tools` on debian-kind system. `i2cdetect -y 1` sample output:
    ```
         0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:          -- -- -- -- -- -- -- -- -- -- -- -- --
    10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    70: -- -- -- -- -- -- 76 --    
    ```


Contact
-------

Please use [Github issue tracker](https://github.com/d2r2/go-vl53l0x/issues) for filing bugs or feature requests.


License
-------

Go-vl53l0x is licensed under MIT License.
