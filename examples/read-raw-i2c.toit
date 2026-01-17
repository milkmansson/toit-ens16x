// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import log
import ens16x show *

/**
Example of raw-read of ENS160, via I2C.

Purposes:
  Show raw readings in a loop:
  - Display only when new data is ready.

*/

main:
  print
  print

  // Establish Log.
  logger := log.default.with-name "raw-example"
  logger = logger.with-level log.DEBUG-LEVEL

  // Enable and drive I2C
  frequency := 400_000
  sda-pin := gpio.Pin 8
  scl-pin := gpio.Pin 9
  bus := i2c.Bus --sda=sda-pin --scl=scl-pin --frequency=frequency

  // Test to see if ENS16x present.
  ens16x-i2c-address := Ens16x.I2C-ADDRESS
  if not bus.test ens16x-i2c-address:
    logger.error "no ENS160 found"
    return

  // Call/start ENS16x driver.
  ens160-device := bus.device Ens16x.I2C_ADDRESS
  ens160-driver := Ens16x ens160-device --startup-operating-mode=Ens16x.OPMODE-STANDARD --logger=logger
  ens160-driver.set-operating-mode Ens16x.OPMODE-STANDARD

  // Variables.
  start := Time.monotonic-us
  width := 10

  while true:
    elapsed-time := Duration --us=(Time.monotonic-us - start)
    if ens160-driver.is-gpr-data-ready:
      raw-read-4 := "$(ens160-driver.read-gpr-raw-int16 4)".pad --left 5
      if (ens160-driver.model-is Ens16x.ENS160-HW-ID):
        raw-read-1 := "$(ens160-driver.read-gpr-raw-int16 1)".pad --left 5
        print "$(duration-to-string elapsed-time)   - R1: $raw-read-1   - R4: $raw-read-4"
      else:
        print "$(duration-to-string elapsed-time)   - R4: $raw-read-4"
    sleep --ms=250
