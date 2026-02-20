// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import log
import aht20-driver
import ens16x show *

/**
Example of ENS160 operation via I2C, using callbacks.

(This example uses an AHT2x device.  This example provided in support of the
popular ENS160/AHT21 combined modules.)

Purposes:
  Take the temperature from an AHT21 on the same bus, and supply its function
  callback style to the for ENS160 for calibration.  To reduce unnecessary load
  the temperature/humidity callbacks are run once per TTL.

  Show readings in a loop:
  - Keep updating ENS160 with temp/humidity from BME280.
  - Show temperature as known by the ENS160.
  - Show elapsed time.
  - Display only when new data is ready.

*/

main:
  print
  print

  // Establish Log.
  logger := log.default.with-name "example"
  logger = logger.with-level log.DEBUG-LEVEL

  // Enable and drive I2C
  frequency := 400_000
  sda-pin := gpio.Pin 19
  scl-pin := gpio.Pin 20
  bus := i2c.Bus --sda=sda-pin --scl=scl-pin --frequency=frequency

  // Test to see if ENS16x present.
  ens16x-i2c-address := Ens16x.I2C-ADDRESS
  if not bus.test ens16x-i2c-address:
    logger.error "no ENS160 found"
    return
  logger.info "found ENS160" --tags={"address":"0x$(%02x ens16x-i2c-address)"}

  // Call/start ENS16x driver.
  ens160-device := bus.device Ens16x.I2C_ADDRESS
  ens160-driver := Ens16x ens160-device --startup-operating-mode=Ens16x.OPMODE-STANDARD --logger=logger
  ens160-driver.set-operating-mode Ens16x.OPMODE-STANDARD

  // Test to see if AHT21 present, and if so, make use of it.
  aht21-device := null
  aht21-driver := null
  if not bus.test aht20-driver.I2C-ADDRESS:
    logger.error "no AHT20 found"
  else:
    logger.info "found AHT20" --tags={"address":"0x$(%02x aht20-driver.I2C-ADDRESS)"}
    aht21-device = bus.device aht20-driver.I2C-ADDRESS
    aht21-driver = aht20-driver.Driver aht21-device

    // Set the compensation callback to the functions from the AHT21 driver.
    ens160-driver.set-compensation-temp-callback :: aht21-driver.read-temperature
    ens160-driver.set-compensation-humidity-callback :: aht21-driver.read-humidity

    // Set the minimum delay between updates from the callbacks (default is
    // 30 secons, here is set to 20 as an example).
    ens160-driver.set-callback-ttl (Duration --s=20)

  // Variables.
  start := Time.monotonic-us
  width := 10
  compensation-humidity := ""

  // For testing purposes, examine the values in the status register, and
  // report whether they are set.
  if ens160-driver.is-compensation-temp-set:
    logger.info "Manually set compensation temperature: $(%0.2f (ens160-driver.get-compensation-temp)) c"
  else:
    logger.info "Manually set compensation temperature: <not-set>"

  // For testing purposes, examine the values in the status register, and
  // report whether they are set.
  if ens160-driver.is-compensation-humidity-set:
    logger.info "Manually set compensation Humidity: $(%0.2f ens160-driver.get-compensation-humidity) %rh"
  else:
    logger.info "Manually set compensation Humidity: <not-set>"

  // Sleep 1 sec (ENS160 STANDARD opmode updates at 1Hz)
  sleep --ms=1000

  logger.info "Current compensation temperature: $(%0.2f ens160-driver.get-temp) c"
  logger.info "Current compensation Humidity: $(%0.2f ens160-driver.get-humidity) %rh"
  logger.info "Current eCO2: $ens160-driver.read-eco2"
  logger.info "Current TVOC: $ens160-driver.read-tvoc"
  logger.info "Current AQI(UBA): $ens160-driver.read-aqi-uba"

  while true:
    elapsed-time := Duration --us=(Time.monotonic-us - start)
    if ens160-driver.is-data-ready:
      temp := "$(%0.2f ens160-driver.get-temp)c".pad --left width
      humidity := "$(%0.2f ens160-driver.get-humidity)%rh".pad --left width
      eco2 := "$ens160-driver.read-eco2".pad --left width
      tvoc := "$ens160-driver.read-tvoc".pad --left width
      aqi-uba := "$ens160-driver.read-aqi-uba".pad --left 3
      print "$(duration-to-string elapsed-time) - $temp $humidity $eco2 $tvoc $aqi-uba"
    sleep --ms=250
