// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import log
import ens16x show *
import bme280

/**
Example of ENS160 operation.

Purposes:
  Take the temperature from a BME280, supply it for ENS160 calibration.
  Show readings in a loop:
  - Keep updating ENS160 with temp/humidity from BME280.
  - Show temperature as known by the ENS160.
  - Show elapsed time.
  - Display only when new data is ready.

Code will still run if BME280 not present.  ENS160 will simply return its
  default values for temperature and humidity calibration.

*/

main:
  print
  print

  // Establish Log.
  logger := log.default.with-name "example"
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
  logger.info "found ENS160" --tags={"address":"0x$(%02x ens16x-i2c-address)"}

  // Call/start ENS16x driver.
  ens160-device := bus.device Ens16x.I2C_ADDRESS
  ens160-driver := Ens16x ens160-device --startup-operating-mode=Ens16x.OPMODE-STANDARD --logger=logger
  ens160-driver.set-operating-mode Ens16x.OPMODE-STANDARD

  // Test to see if BME280 present.
  bme280-device := null
  bme280-driver := null
  if not bus.test bme280.I2C-ADDRESS:
    logger.error "no BME280 found"
  else:
    logger.info "found BME280" --tags={"address":"0x$(%02x bme280.I2C-ADDRESS)"}
    bme280-device = bus.device bme280.I2C-ADDRESS
    bme280-driver = bme280.Driver bme280-device
    ens160-driver.set-compensation-temp bme280-driver.read-temperature
    ens160-driver.set-compensation-humidity bme280-driver.read-humidity

  compensation-temp := ""
  // For testing purposes, examine the values in the status register
  if ens160-driver.get-compensation-temp:
    compensation-temp = "$(%0.2f (ens160-driver.get-compensation-temp)) c"
  else:
    compensation-temp = "<Not Set>"

  compensation-humidity := ""
  // For testing purposes, examine the values in the status register
  if ens160-driver.get-compensation-humidity:
    compensation-humidity = "$(%0.2f ens160-driver.get-compensation-humidity) %rh"
  else:
    compensation-humidity = "<Not Set>"

  // Sleep 1 sec (ENS160 STANDARD opmode updates at 1Hz)
  sleep --ms=1000

  logger.info "Manually set compensation temperature: $compensation-temp"
  logger.info "Manually set compensation Humidity: $compensation-humidity"
  logger.info "Current compensation temperature: $(%0.2f ens160-driver.get-temp)"
  logger.info "Current compensation Humidity: $(%0.2f ens160-driver.get-humidity)"
  logger.info "Current eCO2: $ens160-driver.read-eco2"
  logger.info "Current TVOC: $ens160-driver.read-tvoc"
  logger.info "Current AQI(UBA): $ens160-driver.read-aqi-uba"

  start := Time.monotonic-us
  width := 10

  while true:
    elapsed-time := Duration --us=(Time.monotonic-us - start)
    if ens160-driver.is-data-ready:
      if bme280-driver:
        ens160-driver.set-compensation-temp bme280-driver.read-temperature
        ens160-driver.set-compensation-humidity bme280-driver.read-humidity

      temp := "$(%0.2f ens160-driver.get-temp)c".pad --left width
      humidity := "$(%0.2f ens160-driver.get-humidity)%rh".pad --left width
      eco2 := "$ens160-driver.read-eco2".pad --left width
      tvoc := "$ens160-driver.read-tvoc".pad --left width
      aqi-uba := "$ens160-driver.read-aqi-uba".pad --left 3
      print "$(duration-to-string elapsed-time) - $temp $humidity $eco2 $tvoc $aqi-uba"
    sleep --ms=250
