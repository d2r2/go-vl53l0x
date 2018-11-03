package vl53l0x

import logger "github.com/d2r2/go-logger"

// You can manage verbosity of log output
// in the package by changing last parameter value.
var lg = logger.NewPackageLogger("vl53l0x",
	logger.DebugLevel,
	// logger.InfoLevel,
)
