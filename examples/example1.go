package main

import (
	"context"
	"os"
	"syscall"

	i2c "github.com/d2r2/go-i2c"
	logger "github.com/d2r2/go-logger"
	shell "github.com/d2r2/go-shell"
	vl53l0x "github.com/d2r2/go-vl53l0x"
)

var lg = logger.NewPackageLogger("main",
	logger.DebugLevel,
	// logger.InfoLevel,
)

func main() {
	defer logger.FinalizeLogger()
	// Create new connection to i2c-bus on 1 line with address 0x40.
	// Use i2cdetect utility to find device address over the i2c-bus
	i2c, err := i2c.NewI2C(0x29, 0)
	if err != nil {
		lg.Fatal(err)
	}
	defer i2c.Close()

	lg.Notify("**********************************************************************************************")
	lg.Notify("*** !!! READ THIS !!!")
	lg.Notify("*** You can change verbosity of output, by modifying logging level of modules \"i2c\", \"vl53l0x\".")
	lg.Notify("*** Uncomment/comment corresponding lines with call to ChangePackageLogLevel(...)")
	lg.Notify("*** !!! READ THIS !!!")
	lg.Notify("**********************************************************************************************")
	// Uncomment/comment next line to suppress/increase verbosity of output
	logger.ChangePackageLogLevel("i2c", logger.InfoLevel)
	logger.ChangePackageLogLevel("vl53l0x", logger.InfoLevel)

	sensor := vl53l0x.NewVl53l0x()
	lg.Notify("**********************************************************************************************")
	lg.Notify("*** Reset/initialize sensor")
	lg.Notify("**********************************************************************************************")
	err = sensor.Reset(i2c)
	if err != nil {
		lg.Fatalf("Error reseting sensor: %s", err)
	}
	// It's highly recommended to reset sensor before repeated initialization.
	// By default, sensor initialized with "RegularRange" and "RegularAccuracy" parameters.
	err = sensor.Init(i2c)
	if err != nil {
		lg.Fatalf("Failed to initialize sensor: %s", err)
	}
	rev, err := sensor.GetProductMinorRevision(i2c)
	if err != nil {
		lg.Fatalf("Error getting sensor minor revision: %s", err)
	}
	lg.Infof("Sensor minor revision = %d", rev)

	lg.Notify("**********************************************************************************************")
	lg.Notify("*** Сonfigure sensor")
	lg.Notify("**********************************************************************************************")
	rngConfig := vl53l0x.RegularRange
	speedConfig := vl53l0x.GoodAccuracy
	lg.Infof("Configure sensor with  %q and %q",
		rngConfig, speedConfig)
	err = sensor.Config(i2c, rngConfig, speedConfig)
	if err != nil {
		lg.Fatalf("Failed to initialize sensor: %s", err)
	}

	lg.Notify("**********************************************************************************************")
	lg.Notify("*** Single shot range measurement mode")
	lg.Notify("**********************************************************************************************")
	rng, err := sensor.ReadRangeSingleMillimeters(i2c)
	if err != nil {
		lg.Fatalf("Failed to measure range: %s", err)
	}
	lg.Infof("Measured range = %v mm", rng)

	lg.Notify("**********************************************************************************************")
	lg.Notify("*** Continuous shot range measurement mode")
	lg.Notify("**********************************************************************************************")
	var freq uint32 = 100
	times := 20
	lg.Infof("Made measurement each %d milliseconds, %d times", freq, times)
	err = sensor.StartContinuous(i2c, freq)
	if err != nil {
		lg.Fatalf("Can't start continuous measures: %s", err)
	}
	// create context with cancellation possibility
	ctx, cancel := context.WithCancel(context.Background())
	// use done channel as a trigger to exit from signal waiting goroutine
	done := make(chan struct{})
	defer close(done)
	// build actual signals list to control
	signals := []os.Signal{os.Kill, os.Interrupt}
	if shell.IsLinuxMacOSFreeBSD() {
		signals = append(signals, syscall.SIGTERM)
	}
	// run goroutine waiting for OS termination events, including keyboard Ctrl+C
	shell.CloseContextOnSignals(cancel, done, signals...)

	for i := 0; i < times; i++ {
		rng, err = sensor.ReadRangeContinuousMillimeters(i2c)
		if err != nil {
			lg.Fatalf("Failed to measure range: %s", err)
		}
		lg.Infof("Measured range = %v mm", rng)
		select {
		// Check for termination request.
		case <-ctx.Done():
			err = sensor.StopContinuous(i2c)
			if err != nil {
				lg.Fatal(err)
			}
			lg.Fatal(ctx.Err())
		default:
		}
	}
	err = sensor.StopContinuous(i2c)
	if err != nil {
		lg.Fatalf("Error stopping continuous measures: %s", err)
	}

	lg.Notify("**********************************************************************************************")
	lg.Notify("*** Reconfigure sensor")
	lg.Notify("**********************************************************************************************")
	rngConfig = vl53l0x.RegularRange
	speedConfig = vl53l0x.RegularAccuracy
	lg.Infof("Reconfigure sensor with %q and %q",
		rngConfig, speedConfig)
	err = sensor.Config(i2c, rngConfig, speedConfig)
	if err != nil {
		lg.Fatalf("Failed to initialize sensor: %s", err)
	}

	lg.Notify("**********************************************************************************************")
	lg.Notify("*** Single shot range measurement mode")
	lg.Notify("**********************************************************************************************")
	rng, err = sensor.ReadRangeSingleMillimeters(i2c)
	if err != nil {
		lg.Fatalf("Failed to measure range: %s", err)
	}
	lg.Infof("Measured range = %v mm", rng)

}
