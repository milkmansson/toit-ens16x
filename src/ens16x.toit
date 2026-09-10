// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by an MIT-style license that can be
// found in the package's LICENSE file.

import io show LITTLE-ENDIAN
import log
import math
import monitor
import serial.device as serial
import serial.registers as registers

class Ens16x:
  static I2C-ADDRESS          ::= 0x53
  static I2C-ADDRESS-ALT      ::= 0x52  // When pin MISO/ADDR is low.

  // Registers.
  static REG-PART-ID_ ::= 0x00  // 2, R  Device Identity 0x60, 0x01.
  static REG-OPMODE_  ::= 0x10  // 1, RW Operating Mode.
  static REG-CFG_     ::= 0x11  // 1, RW Interrupt Pin Configuration.
  static REG-CMD_     ::= 0x12  // 1, RW Additional System Commands.
  static REG-TEMP-IN_ ::= 0x13  // 2, RW Host Ambient Temperature Information.
  static REG-RH-IN_   ::= 0x15  // 2, RW Host Relative Humidity Information.

  static REG-STATUS_        ::= 0x20  // 1, R Device Status.
  static REG-DATA-AQI-UBA_  ::= 0x21  // 1, R Air Quality Index (UBA).
  static REG-DATA-TVOC_     ::= 0x22  // 2, R TVOC Concentration (ppb).
  static REG-DATA-ETOH_     ::= 0x22  // 2, R Mirror of TVOC for ETOH Concentration (ppb).
  static REG-DATA-ECO2_     ::= 0x24  // 2, R Equivalent CO2 Concentration (ppm).
  static REG-DATA-AQI-S_    ::= 0x26  // 2, R Relative Air Quality Index (ScioSense).

  static REG-DATA-T_    ::= 0x30  // 2, R Temperature used in calculations.
  static REG-DATA-RH_   ::= 0x32  // 2, R Relative Humidity used in calculations.
  static REG-DATA-MISR_ ::= 0x38  // 1, R Data Integrity Field (optional).

  static REG-GPR-WRITE-BASE_ ::= 0x40  // 0x40..0x47: General Purpose Write Registers.
  static REG-GPR-READ-BASE_  ::= 0x48  // 0x48..0x4F: General Purpose Read Registers.

  // REG-OPMODE_: Configuration of Operating Modes.
  static OPMODE-DEEPSLEEP     ::= 0x00  // DEEP SLEEP mode (low-power standby).
  static OPMODE-IDLE          ::= 0x01  // IDLE mode (low power).
  static OPMODE-STANDARD      ::= 0x02  // STANDARD Gas Sensing Mode (1 sample/sec).
  static OPMODE-LOWPOWER      ::= 0x03  // LOW POWER gas sensing mode (1/1min). ENS161 only.
  static OPMODE-ULT-LOWPOWER  ::= 0x04  // ULTRA LOW POWER gas sensing mode (1/5min). ENS161 only.
  static OPMODE-RESET         ::= 0xF0  // RESET.
  static OPMODES_/Map ::= {
    OPMODE-DEEPSLEEP: "OPMODE-DEEPSLEEP",
    OPMODE-IDLE: "OPMODE-IDLE",
    OPMODE-STANDARD: "OPMODE-STANDARD",
    OPMODE-LOWPOWER: "OPMODE-LOWPOWER",
    OPMODE-ULT-LOWPOWER: "OPMODE-ULT-LOWPOWER",
    OPMODE-RESET: "OPMODE-RESET",
  }
  static OPMODES-ENS161-ONLY_/Set ::= {OPMODE-LOWPOWER, OPMODE-ULT-LOWPOWER}

  // REG-CFG_: Interrupt Pin Operation.
  static CFG-INT-POL-MASK_   ::= 0b01000000  // RW.
  static CFG-INT-DRIVE-MASK_ ::= 0b00100000  // RW.
  static CFG-INT-GPRR-MASK_  ::= 0b00010000  // RW Asserts if new data is in GPRR Registers.
  static CFG-INT-DAT-MASK_   ::= 0b00000010  // RW Asserts if new data is in DATA-XXX Registers.
  static CFG-INT-EN-MASK_    ::= 0b00000001  // Enables the interrupt pin.

  // REG-CMD_: Additional Commands.
  static CMD-NOP_        ::= 0x00
  static CMD-GET-APPVER_ ::= 0x0E
  static CMD-CLR-GPR_    ::= 0xCC

  // REG-STATUS_: Masks.
  static STATUS-OPMODE-RUNNING-MASK_ ::= 0b10000000
  static STATUS-OPMODE-ERROR-MASK_   ::= 0b01000000
  static STATUS-OUTPUT-VALID-MASK_   ::= 0b00001100
  static STATUS-NEW-DATA-MASK_       ::= 0b00000010
  static STATUS-NEW-GPR-MASK_        ::= 0b00000001

  // REG-STATUS_: Data-validity values.
  static OUTPUT-NORMAL_         ::= 0b00  // OUTPUT is valid.
  static OUTPUT-WARM-UP_        ::= 0b01  // OUTPUT is valid but WARMUP (?).
  static OUTPUT-INIT-START-UP_  ::= 0b10  // ENS160 only.
  static OUTPUT-INVALID_        ::= 0b11

  // REG-DATA-AQI-UBA_ & REG-DATA-AQI-S_: Read masks.
  static AQI-UBA-MASK_ ::= 0b00000111  // Datasheet says 0:2, even if too large.

  // System Timings: Appear to refer to waits after specific commands.
  static TIMING-RESET_            ::= Duration --ms=50
  static TIMING-STANDARD-MEASURE_ ::= Duration --ms=1000
  static TIMING-CLEAR-GPR_        ::= Duration --ms=2
  static TIMING-TIMEOUT_          ::= Duration --ms=5000

  // MISR: Checksum verification on register reads.
  // The polynomial used in the CRC computation in REG-DATA-MISR_, 76543210 bit weight factor.
  // 0b00011101 = x^8+x^4+x^3+x^2+x^0 (x^8 is implicit).
  static MISR-POLY_ ::= 0b00011101  // (0x1D).

  static RAW-RESISTANCE-LOG2-SCALE_ ::= 2048.0

  // $write-register_ statics for bit width. All 16 bit read/writes are LE.
  static WIDTH-8_ ::= 8
  static WIDTH-16_ ::= 16
  static DEFAULT-REGISTER-WIDTH_ ::= WIDTH-8_

  static ENS160-HW-ID ::= 0x160
  static ENS161-HW-ID ::= 0x161
  static HW-IDS_ ::= {
    ENS160-HW-ID: "ENS160",
    ENS161-HW-ID: "ENS161",
  }

  // Software tracking of CRC value (updated by $misr-update-software_).
  misr_/int := 0

  // Serialises resync/read/compare in $read-register_ so that concurrent
  // readers cannot interleave and produce false CRC failures.
  misr-mutex_/monitor.Mutex ::= monitor.Mutex

  // Detected HW version for function use. Mutable only because it is set
  // via a method call in the constructor.
  hw-id_/int := 0
  reg_/registers.Registers
  logger_/log.Logger

  // Lambdas for storing temperature/humidity compensation callbacks, and TTL.
  temp-comp-callback_/Lambda? := null
  temp-comp-callback-ts_/int? := null
  humidity-comp-callback_/Lambda? := null
  humidity-comp-callback-ts_/int? := null
  callback-ttl_/Duration := Duration --s=30

  constructor
      device/serial.Device
      --startup-operating-mode/int=OPMODE-STANDARD
      --logger/log.Logger=log.default:
    assert: OPMODES_.contains startup-operating-mode
    logger_ = logger.with-name "ens16x"
    reg_ = device.registers

    // Check correct HW ID.
    hw-id_ = get-hardware-id
    if not HW-IDS_.contains hw-id_:
      logger_.error "HW ID unsupported" --tags={"hw-id": "0x$(%03x hw-id_)"}
      throw "Incorrect HW ID"

    // Reset device, returning to OPMODE-IDLE.
    reset OPMODE-IDLE

    // Reset SW MISR value as device reset will zero the HW value.
    misr-resync_

    // Report device type detected and firmware.
    firmware := get-firmware-version
    firmware-string := "v$(firmware[0]).$(firmware[1]).$(firmware[2])"
    logger_.info "$(HW-IDS_[hw-id_]) device started" --tags={"hw-id": "0x$(%03x hw-id_)", "firmware": firmware-string}

    // Ensure device is cleanly in IDLE after firmware version query, as CMD
    // operations may leave internal state that delays OPMODE transitions.
    write-register_ REG-OPMODE_ OPMODE-IDLE
    sleep TIMING-RESET_

    // Report if device is not ready to go.
    data-valid := data-validity
    if data-valid == OUTPUT-INIT-START-UP_:
      // Not used/returned on ENS161, but safe to leave this way.
      logger_.warn "device still in 1 hour first power on period"
    else if data-valid == OUTPUT-WARM-UP_:
      logger_.warn "device still in 3 minute warmup period"
    else if data-valid == OUTPUT-INVALID_:
      logger_.error "device reports invalid"
      throw "device reports invalid"

    // Set to operating mode as given to the constructor.
    set-operating-mode startup-operating-mode
    logger_.info "currently in operating mode" --tags={"opmode": OPMODES_[get-operating-mode]}

    if is-error:
      logger_.error "currently in ERROR condition"

  /**
  Returns the value of the HARDWARE-ID register.
  */
  get-hardware-id -> int:
    return read-register_ REG-PART-ID_ --width=WIDTH-16_

  /**
  Returns the firmware version (APPVER) as a list of three ints.

  The device only answers this command in $OPMODE-IDLE, so the current mode
    is saved, IDLE is entered, and the original mode is restored afterwards,
    including when the poll for $is-gpr-data-ready times out.

  Throws if the device does not report GPR data ready within
    $TIMING-TIMEOUT_.
  */
  get-firmware-version -> List:
    original-mode := get-operating-mode
    out-bytes := [0x00, 0x00, 0x00]

    duration := Duration.ZERO
    exception := null
    try:
      if original-mode != OPMODE-IDLE: set-operating-mode OPMODE-IDLE
      // Poll to determine if data is ready.
      exception = catch --unwind=(: it != DEADLINE-EXCEEDED-ERROR):
        with-timeout TIMING-TIMEOUT_:
          duration = Duration.of:
            cmd-no-op_
            cmd-clear-gpr_
            cmd-get-appver_
            while not is-gpr-data-ready:
              sleep --ms=25
            out-bytes[0] = read-register_ (REG-GPR-READ-BASE_ + 4)
            out-bytes[1] = read-register_ (REG-GPR-READ-BASE_ + 5)
            out-bytes[2] = read-register_ (REG-GPR-READ-BASE_ + 6)
    finally:
      if original-mode != OPMODE-IDLE: set-operating-mode original-mode

    if exception:
      logger_.error "get-firmware-version - wait for is-gpr-data-ready timed out" --tags={"duration": duration.in-ms}
      throw "get-firmware-version timeout"

    return out-bytes

  cmd-get-appver_ -> none:
    write-register_ REG-CMD_ CMD-GET-APPVER_
    // No timed wait here; callers check against $is-gpr-data-ready instead.

  cmd-clear-gpr_ -> none:
    write-register_ REG-CMD_ CMD-CLR-GPR_
    sleep TIMING-CLEAR-GPR_

  cmd-no-op_ -> none:
    // Don't know why we do this, however it is done in ScioSense's examples.
    write-register_ REG-CMD_ CMD-NOP_

  /**
  Resets the device.

  A normal reset would put the device into $OPMODE-DEEPSLEEP. This function
    defaults to $OPMODE-IDLE unless $mode is supplied. After the reset
    command, this function waits until the device reports no opmode running
    before proceeding to set the target mode.
  */
  reset mode/int=OPMODE-IDLE -> none:
    write-register_ REG-OPMODE_ OPMODE-RESET
    sleep TIMING-RESET_

    // Wait for the device to finish its reset cycle - STATAS should go low.
    duration := Duration.ZERO
    exception := catch --unwind=(: it != DEADLINE-EXCEEDED-ERROR):
      with-timeout TIMING-TIMEOUT_:
        duration = Duration.of:
          while is-opmode-running:
            sleep --ms=25

    if exception:
      logger_.error "reset - device did not clear STATAS after reset" --tags={"duration": duration.in-ms}
      throw "reset failed"
    else:
      logger_.debug "reset completed" --tags={"duration": duration.in-ms}

    misr-resync_
    if mode != OPMODE-DEEPSLEEP: set-operating-mode mode

  /**
  Sets the operating mode.

  Must be one of $OPMODE-DEEPSLEEP, $OPMODE-IDLE, $OPMODE-STANDARD, $OPMODE-RESET.
    $OPMODE-LOWPOWER and $OPMODE-ULT-LOWPOWER are also valid, but only on the
    ENS161 (see $model-is). Passing an ENS161-only mode on an ENS160 throws
    before anything is written to the device.

  In $OPMODE-DEEPSLEEP mode, the ENS160 has limited functionality but will respond to
    a change in mode. $OPMODE-IDLE is intended for configuration before running an
    active sensing mode. $OPMODE-STANDARD is the active gas sensing mode.
  */
  set-operating-mode mode/int -> none:
    assert: OPMODES_.contains mode
    if (OPMODES-ENS161-ONLY_.contains mode) and not (model-is ENS161-HW-ID):
      logger_.error "opmode not available on this device" --tags={"mode": OPMODES_[mode], "hw-id": "0x$(%03x hw-id_)"}
      throw "OPMODE not supported on $(HW-IDS_[hw-id_])"

    current-mode := get-operating-mode

    if current-mode == mode:
      logger_.debug "operating mode already set (doing nothing)" --tags={"opmode": OPMODES_[mode]}
      return

    // Setting OPMODE to IDLE first. Return if IDLE was the target.
    if current-mode != OPMODE-IDLE and current-mode != OPMODE-RESET:
      write-register_ REG-OPMODE_ OPMODE-IDLE
      sleep TIMING-RESET_
      // Note: IDLE is not polled via $is-opmode-running below. Whether the
      // STATAS bit is set while in IDLE has not been confirmed against the
      // datasheet, so a timed wait is used instead.
      if mode == OPMODE-IDLE: return

    write-register_ REG-OPMODE_ mode
    if mode == OPMODE-RESET:
      sleep TIMING-RESET_
      return

    // $OPMODE-DEEPSLEEP has $is-opmode-running always returning false, so exit.
    if mode == OPMODE-DEEPSLEEP: return

    // Give the device a moment to begin transitioning before polling.
    sleep --ms=20

    // Check for an error condition before entering the poll - an invalid mode
    // selection will never result in is-opmode-running becoming true.
    if is-error:
      logger_.error "device error after opmode write" --tags={"mode": OPMODES_[mode]}
      throw "OPMODE could not be set"

    duration := Duration.ZERO
    exception := catch --unwind=(: it != DEADLINE-EXCEEDED-ERROR):
      with-timeout TIMING-TIMEOUT_:
        duration = Duration.of:
          while not is-opmode-running:
            sleep --ms=100

    if exception:
      logger_.error "set opmode timed out" --tags={"duration": duration.in-ms}
      throw "OPMODE set timeout"
    else:
      logger_.info "set opmode duration" --tags={"mode": OPMODES_[mode], "duration": duration.in-ms}

  /**
  Returns the current operating mode.
  */
  get-operating-mode -> int:
    return read-register_ REG-OPMODE_

  /**
  Sets the ambient temperature used for compensation (in Celsius).

  The register can be written at any time. Set to null to clear the
    configured value (writes raw 0 to the register).

  Note: A raw register value of 0 is used as the null sentinel. This is
    safe because 0 maps to 0K (-273.15°C), which is physically impossible
    and will never be a valid compensation input.
  */
  set-compensation-temp celsius/float? -> none:
    if celsius == null:
      write-register_ REG-TEMP-IN_ 0 --width=WIDTH-16_
      return
    kelvin/float := celsius + 273.15
    raw/int := (kelvin * 64.0).round.to-int
    write-register_ REG-TEMP-IN_ raw --width=WIDTH-16_

  /**
  Returns the ambient temperature configured for compensation (in Celsius).

  Returns null if no compensation temperature has been set. See
    $set-compensation-temp.
  */
  get-compensation-temp -> float?:
    raw := read-register_ REG-TEMP-IN_ --width=WIDTH-16_
    if raw == 0: return null
    return (raw.to-float / 64.0) - 273.15

  /**
  Whether a custom temperature is set for calculation calibration.

  See $set-compensation-temp.
  */
  is-compensation-temp-set -> bool:
    raw := read-register_ REG-TEMP-IN_ --width=WIDTH-16_
    return raw != 0

  /**
  Sets the function providing temperature values for compensation.

  Useful for ENS160 modules that have built in AHT21, or where the project also
    contains a temperature sensor. Disabled by default. Set to `null` to
    disable.
  */
  set-compensation-temp-callback callback/Lambda? -> none:
    temp-comp-callback_ = callback
    if temp-comp-callback_:
      set-compensation-temp temp-comp-callback_.call
      temp-comp-callback-ts_ = Time.monotonic-us

  /**
  Sets the relative humidity used for compensation (in %RH).

  The register can be written at any time. Set to null to clear the
    configured value (writes raw 0 to the register).

  Note: A raw register value of 0 is used as a null sentinel. This is
    safe because 0 maps to 0%RH, which the device's own recommended
    operating range (20–80%RH) excludes as a meaningful input.
  */
  set-compensation-humidity rh/float? -> none:
    if rh == null:
      write-register_ REG-RH-IN_ 0 --width=WIDTH-16_
      return
    raw := (rh * 512).round.to-int
    write-register_ REG-RH-IN_ raw --width=WIDTH-16_

  /**
  Returns the relative humidity configured for compensation (in %RH).

  Returns null if no compensation humidity has been set. See
    $set-compensation-humidity.
  */
  get-compensation-humidity -> float?:
    raw := read-register_ REG-RH-IN_ --width=WIDTH-16_
    if raw == 0: return null
    return raw.to-float / 512.0

  /**
  Whether a custom humidity is set for calculation calibration.

  See $set-compensation-humidity.
  */
  is-compensation-humidity-set -> bool:
    raw := read-register_ REG-RH-IN_ --width=WIDTH-16_
    return raw != 0

  /**
  Sets the function providing humidity values for compensation.

  Useful for ENS160 modules that have built in AHT21, or where the project also
    contains a humidity sensor. Disabled by default. Set to `null` to disable.
  */
  set-compensation-humidity-callback callback/Lambda? -> none:
    humidity-comp-callback_ = callback
    if humidity-comp-callback_:
      set-compensation-humidity humidity-comp-callback_.call
      humidity-comp-callback-ts_ = Time.monotonic-us

  /**
  Updates the compensation values if it is time to do so.
  */
  update-if-necessary_ -> none:
    if humidity-comp-callback_ and (Time.monotonic-us >= (humidity-comp-callback-ts_ + callback-ttl_.in-us)):
      set-compensation-humidity humidity-comp-callback_.call
      humidity-comp-callback-ts_ = Time.monotonic-us
    if temp-comp-callback_ and (Time.monotonic-us >= (temp-comp-callback-ts_ + callback-ttl_.in-us)):
      set-compensation-temp temp-comp-callback_.call
      temp-comp-callback-ts_ = Time.monotonic-us

  /**
  Sets the delay between callback sensor reads.

  Given that temperature reads from ENS160 in typical scenarios do not typically
    change frequently or by large changes, this reduces the read load
    considerably to the temp/humidity sensor, instead of being 1:1 with each
    individual read instruction. (Useful only if using
    $set-compensation-humidity-callback or $set-compensation-temp-callback.)
  */
  set-callback-ttl ttl/Duration -> none:
    callback-ttl_ = ttl

  /** Whether an OPMODE is running. */
  is-opmode-running -> bool:
    return (read-register_ REG-STATUS_ --mask=STATUS-OPMODE-RUNNING-MASK_) == 1

  /**
  Whether an error is detected.

  E.g. Invalid Operating Mode selected. The meaning of the errors may be
    different, depending on the operation being undertaken.
  */
  is-error -> bool:
    return (read-register_ REG-STATUS_ --mask=STATUS-OPMODE-ERROR-MASK_) == 1

  /**
  Whether new data is available in the DATA-x registers.

  Cleared automatically at first DATA-x read.
  */
  is-data-ready -> bool:
    return (read-register_ REG-STATUS_ --mask=STATUS-NEW-DATA-MASK_) == 1

  /**
  Whether new data is available in the GPR-x registers.

  Cleared automatically when any GPR-x register is read.
  */
  is-gpr-data-ready -> bool:
    return (read-register_ REG-STATUS_ --mask=STATUS-NEW-GPR-MASK_) == 1

  /**
  Returns the output data validity.

  The device needs an initial warm up time from the very first power on. In
    addition, each time the device is powered on it requires a 3 minute warm up
    period. These values return which state the device is in.

  Returns one of $OUTPUT-NORMAL_ (normal operation), $OUTPUT-WARM-UP_ (still in
    the 3 minute warm up period), $OUTPUT-INIT-START-UP_ (still in the first
    run, 1 hour initialisation period) and $OUTPUT-INVALID_ (data is invalid).
  */
  data-validity -> int:
    return read-register_ REG-STATUS_ --mask=STATUS-OUTPUT-VALID-MASK_

  /** Whether $data-validity == $OUTPUT-NORMAL_. */
  is-data-valid -> bool:
    return data-validity == OUTPUT-NORMAL_

  /** Returns the Air Quality Index [1..5] as per UBA guidelines. */
  read-aqi-uba -> int:
    update-if-necessary_
    return read-register_ REG-DATA-AQI-UBA_ --mask=AQI-UBA-MASK_ --misr

  /** Returns the total volatile organic compounds (ppb). */
  read-tvoc -> int:
    update-if-necessary_
    return read-register_ REG-DATA-TVOC_ --width=WIDTH-16_ --misr

  /** Returns the equivalent CO2 (ppm). */
  read-eco2 -> int:
    update-if-necessary_
    return read-register_ REG-DATA-ECO2_ --width=WIDTH-16_ --misr

  /**
  Returns the ScioSense air quality index rate of change [0..100].

  Only available on the ENS161; throws on other devices.
  */
  read-aqi-s -> int:
    if not (model-is ENS161-HW-ID):
      logger_.error "aqi-s not available on this device" --tags={"hw-id": "0x$(%03x hw-id_)"}
      throw "AQI-S not available on $(HW-IDS_[hw-id_])"
    update-if-necessary_
    return read-register_ REG-DATA-AQI-S_ --width=WIDTH-16_ --misr

  /**
  Returns the temperature used in calculations (degrees Celsius).

  Temp is taken from $set-compensation-temp, if supplied.
  */
  get-temp -> float:
    raw := read-register_ REG-DATA-T_ --width=WIDTH-16_
    return (raw.to-float / 64.0) - 273.15

  /**
  Returns the humidity used in calculations (%RH).

  Humidity is taken from $set-compensation-humidity, if supplied.
  */
  get-humidity -> float:
    raw := read-register_ REG-DATA-RH_ --width=WIDTH-16_
    return raw.to-float / 512.0

  // Derived measures.

  /** Returns the equivalent ethanol (ppb) value. */
  read-etoh -> int:
    update-if-necessary_
    return read-register_ REG-DATA-ETOH_ --width=WIDTH-16_ --misr

  /**
  Updates the software instance of the rolling CRC counter.

  The documentation says that the hardware register $REG-DATA-MISR_ is updated
    with every read from a register in the range 0x20 to 0x37, using a CRC
    polynomial (POLY). In testing it appears that this register is updated for
    every read from the device, regardless of 8 or 16 bit, all except for the
    MISR register itself. The $read-register_ function calls this function
    once for each individual byte read when `--misr` is given. This keeps the
    internal variable $misr_ in sync with the hardware register. Comparing the
    hardware and software CRC allows one to determine if any data reads have
    become corrupt.
  */
  misr-update-software_ data/int -> none:
    assert: 0 <= data <= 255
    misr-xor := ((misr_ << 1) ^ data) & 0xFF
    if (misr_ & 0x80) == 0:
      misr_ = misr-xor
    else:
      misr_ = misr-xor ^ MISR-POLY_

  /**
  Returns the hardware's rolling CRC counter.

  Uses a direct read so as to prevent an 'infinite loop'. (Where the MISR
    register read triggers a MISR register read for comparing the CRC's, and
    so on.)
  */
  misr-hardware_ -> int:
    return (reg_.read-bytes REG-DATA-MISR_ 1)[0]

  /**
  Whether the hardware MISR and software MISR are equal, i.e. that no
    data read corruptions have occurred.

  When a data read transaction is completed, read $REG-DATA-MISR_, and compare
    it with the software $misr_. They should be equal. If not there is a CRC
    error: one or more bytes were corrupted in the transfer.
  */
  misr-valid_ -> bool:
    return misr_ == misr-hardware_

  /**
  Resets the software MISR value.

  Once the CRC is wrong (or read transactions have been executed without calling
    $misr-update-software_) the software MISR will be out of sync with
    $REG-DATA-MISR_. Because the CRC uses the previous value in the calculation
    it remains out of sync and every read will appear to be a failure. This
    function reads $REG-DATA-MISR_ and stores it in $misr_ to bring the two
    values back in sync.
  */
  misr-resync_ -> none:
    misr_ = misr-hardware_

  /**
  Returns whether the detected hardware matches the given $model.

  Use with the class constants $ENS160-HW-ID and $ENS161-HW-ID to
    distinguish device capabilities at runtime. For example, $read-aqi-s
    and $OPMODE-LOWPOWER are only available on the ENS161.
  */
  model-is model/int -> bool:
    return hw-id_ == model

  /**
  Returns the raw uint16 value of a general purpose register (GPR).

  ENS160 datasheet specifies 'Sensor' 1 as R1, and 'Sensor 4' as R4. The ENS161
    datasheet specifies 'Sensor 4' as R3 - however, the bits and registers are
    the same as R4 in the ENS160 datasheet. To avoid confusion, input to this
    function is the $sensor number instead of the Rx value given in the
    datasheets.

  This function does not call the private $update-if-necessary_ function that
    updates the temperature and humidity compensation values via the callbacks.
  */
  read-gpr-raw-int16 sensor/int -> int:
    assert: 1 <= sensor <= 4
    if sensor == 2 or sensor == 3:
      logger_.warn "sensor not available according to datasheet" --tags={"sensor": sensor, "hw-id": "0x$(%03x hw-id_)"}
    else if sensor == 1 and model-is ENS161-HW-ID:
      logger_.warn "sensor not available according to datasheet" --tags={"sensor": sensor, "hw-id": "0x$(%03x hw-id_)"}
    reg := sensor - 1
    return read-register_ (REG-GPR-READ-BASE_ + (reg * 2)) --width=WIDTH-16_

  /**
  Converts a raw sensor value to resistance in Ohms.

  Both the ENS160 and ENS161 datasheets specify the conversion as
    'Ri-res[Ω] = 2^(Ri-raw / 2048)' where Ri-raw is the unsigned 16-bit
    value obtained from $read-gpr-raw-int16.
  */
  static raw-to-resistance raw/int -> float:
    return math.pow 2.0 (raw.to-float / RAW-RESISTANCE-LOG2-SCALE_)

  /**
  Reads a sensor's raw value and returns the resistance in Ohms.

  Convenience wrapper that calls $read-gpr-raw-int16 and converts the
    result with $raw-to-resistance. See $read-gpr-raw-int16 for which
    $sensor numbers are valid on each device variant.
  */
  read-sensor-resistance sensor/int -> float:
    return raw-to-resistance (read-gpr-raw-int16 sensor)

  /**
  Reads and optionally masks/parses register data.

  Little Endian only. Uses `.read-bytes` so that each individual byte can
    be fed to the software MISR calculation when $misr is true.

  When $misr is true the software CRC is resynced before the read, updated
    with every byte returned, and then compared against the hardware
    $REG-DATA-MISR_ register. A mismatch is logged as an error. The whole
    sequence is guarded by $misr-mutex_ so concurrent readers do not corrupt
    the software CRC. Callers that do not pass `--misr` get no CRC overhead.
  */
  read-register_ register/int -> int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false
      --misr/bool=false:
    assert: (width == 8) or (width == 16)

    if mask == null:
      mask = (width == 8) ? 0xFF : 0xFFFF
    if offset == null:
      offset = mask.count-trailing-zeros

    byte-count := (width == 8) ? 1 : 2
    raw/ByteArray := #[]
    if misr:
      misr-mutex_.do:
        // Resync before the read so that any preceding untracked reads (STATUS
        // polling, GPR reads, etc.) do not leave the software CRC out of step.
        misr-resync_
        raw = reg_.read-bytes register byte-count
        raw.do: misr-update-software_ it
        if not misr-valid_:
          logger_.error "CRC failed" --tags={"hw": "0x$(%02x misr-hardware_)", "sw": "0x$(%02x misr_)"}
    else:
      raw = reg_.read-bytes register byte-count

    register-value/int := 0
    if width == 8:
      register-value = signed ? (LITTLE-ENDIAN.int8 raw 0) : (LITTLE-ENDIAN.uint8 raw 0)
    else:
      register-value = signed ? (LITTLE-ENDIAN.int16 raw 0) : (LITTLE-ENDIAN.uint16 raw 0)

    if ((mask == 0xFFFF) or (mask == 0xFF)) and (offset == 0):
      return register-value
    return (register-value & mask) >> offset

  /**
  Writes register data - either masked or full register writes.

  Little Endian only. No modifications are required or made to support MISR.
  */
  write-register_ register/int value/int -> none
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false:
    assert: (width == 8) or (width == 16)
    if mask == null:
      mask = (width == 8) ? 0xFF : 0xFFFF
    if offset == null:
      offset = mask.count-trailing-zeros

    field-mask/int := (mask >> offset)
    assert: ((value & ~field-mask) == 0)  // Fit check.

    // Full-width direct write.
    if ((width == 8) and (mask == 0xFF) and (offset == 0)) or
        ((width == 16) and (mask == 0xFFFF) and (offset == 0)):
      write-raw_ register value --width=width --signed=signed
      return

    // Read register for modification.
    old-value/int := 0
    if width == 8:
      old-value = signed ? (reg_.read-i8 register) : (reg_.read-u8 register)
    else:
      old-value = signed ? (reg_.read-i16-le register) : (reg_.read-u16-le register)

    new-value/int := (old-value & ~mask) | (value << offset)
    write-raw_ register new-value --width=width --signed=signed

  /** Writes a full 8 or 16 bit value to $register without masking. */
  write-raw_ register/int value/int --width/int --signed/bool -> none:
    if width == 8:
      if signed:
        reg_.write-i8 register value
      else:
        reg_.write-u8 register value
    else:
      if signed:
        reg_.write-i16-le register value
      else:
        reg_.write-u16-le register value

// Debug and test helpers. Not used by the driver itself.

/**
Returns a string displaying the bits of $x grouped for readability.

The output is left-padded with zeros to at least $min-display-bits and then
  rounded up to a whole number of groups of $group-size bits, separated by
  $sep.
*/
bits-grouped x/int -> string
    --min-display-bits/int=0
    --group-size/int=4
    --sep/string=".":
  assert: x >= 0
  assert: group-size > 0

  // Raw binary.
  bin := "$(%b x)"

  // Choose target width: at least min-display-bits, then round up to a full group.
  width := bin.size
  if min-display-bits > width:
    width = min-display-bits
  if group-size > width:
    width = group-size
  leftover := width % group-size
  if leftover > 0:
    width = width + (group-size - leftover)

  // Left-pad to target width.
  bin = bin.pad --left width '0'

  // Group left to right.
  out := ""
  i := 0
  while i < bin.size:
    if i > 0: out = "$(out)$(sep)"
    j := i + group-size
    if j > bin.size: j = bin.size
    out = "$(out)$(bin[i..j])"
    i = j

  return out

/**
Formats $dur as Dd HH:MM:SS.mmm, HH:MM:SS.mmm, MM:SS.mmm or SS.mmm,
  left-padded to $width characters.
*/
duration-to-string dur/Duration --width/int=12 -> string:
  total-ms := dur.in-ms
  sign := ""
  if total-ms < 0:
    sign = "-"
    total-ms = -total-ms

  ms/int := total-ms % 1000
  total-s := total-ms / 1000

  s/int := total-s % 60
  total-m/int := total-s / 60

  m/int := total-m % 60
  total-h := total-m / 60

  h/int := total-h % 24
  total-d := total-h / 24

  if total-d > 0:
    return "$sign$(total-d)d $(%02d h):$(%02d m):$(%02d s).$(%03d ms)".pad --left width
  if h > 0:
    return "$sign$(%02d h):$(%02d m):$(%02d s).$(%03d ms)".pad --left width
  else if m > 0:
    return "$sign$(%02d m):$(%02d s).$(%03d ms)".pad --left width
  else:
    return "$sign$(%01d s).$(%03d ms)".pad --left width
