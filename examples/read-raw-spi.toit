// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import spi
import log
import ens16x show *

/**
Example of raw-read of ENS160, via SPI.

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

  bus := spi.Bus
    --miso=gpio.Pin 7
    --mosi=gpio.Pin 6
    --clock=gpio.Pin 5
  cs-pin := gpio.Pin 15

  // Call/start ENS16x driver.
  ens160-device := bus.device --cs=cs-pin --frequency=200_000
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
