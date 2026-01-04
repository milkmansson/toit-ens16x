// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by an MIT-style license that can be
// found in the package's LICENSE file.

import io show LITTLE-ENDIAN
import log
import serial.device as serial
import serial.registers as registers

class Ens16x:
  static I2C-ADDRESS          ::= 0x53
  static I2C-ADDRESS-ALT      ::= 0x52    // When pin MISO/ADDR is low

  // Registers
  static REG-PART-ID_ ::= 0x00  // 2, R  Device Identity 0x60, 0x01
  static REG-OPMODE_  ::= 0x10  // 1, RW Operating Mode
  static REG-CFG_     ::= 0x11  // 1, RW Interrupt Pin Configuration
  static REG-CMD_     ::= 0x12  // 1, RW Additional System Commands
  static REG-TEMP-IN_ ::= 0x13  // 2, RW Host Ambient Temperature Information
  static REG-RH-IN_   ::= 0x15  // 2, RW Host Relative Humidity Information

  static REG-STATUS_        ::= 0x20  // 1, R Operating Mode
  static REG-DATA-AQI-UBA_  ::= 0x21  // 1, R Air Quality Index (UBA)
  static REG-DATA-TVOC_     ::= 0x22  // 2, R TVOC Concentration (ppb)
  static REG-DATA-ETOH_     ::= 0x22  // 2, R Mirror of TVOC for ETOH Concentration (ppb)
  static REG-DATA-ECO2_     ::= 0x24  // 2, R Equivalent CO2 Concentration (ppm)
  static REG-DATA-AQI-S_    ::= 0x26  // 2, R Relative Air Quality Index (ScioSense)

  static REG-DATA-T_    ::= 0x30  // 2 R Temperature used in calculations
  static REG-DATA-RH_   ::= 0x32  // 2 R Relative Humidity used in calculations
  static REG-DATA-MISR_ ::= 0x38  // 1 R Data Integrity Field (optional)

  static REG-GPR-WRITE-BASE_ ::= 0x40  // 0x40..0x47: General Purpose Write Registers
  static REG-GPR-READ-BASE_  ::= 0x48  // 0x48..0x4F: General Purpose Read Registers

  // REG-REG-OPMODE_: Configuration of Operating Modes.
  static OPMODE-DEEPSLEEP     ::= 0x00 // DEEP SLEEP mode (low-power standby)
  static OPMODE-IDLE          ::= 0x01 // IDLE mode (low power)
  static OPMODE-STANDARD      ::= 0x02 // STANDARD Gas Sensing Mode (1 sample/sec)
  static OPMODE-LOWPOWER      ::= 0x03 // LOW POWER gas sensing mode (1/1min)
  static OPMODE-ULT-LOWPOWER  ::= 0x04 // ULTRA LOW POWER gas sensing mode (1/5min)
  static OPMODE-RESET         ::= 0xF0 // RESET
  static OPMODES_/Map ::= {
    OPMODE-DEEPSLEEP: "OPMODE-DEEPSLEEP",
    OPMODE-IDLE: "OPMODE-IDLE",
    OPMODE-STANDARD: "OPMODE-STANDARD",
    OPMODE-LOWPOWER: "OPMODE-LOWPOWER",          // ENS161 only.
    OPMODE-ULT-LOWPOWER: "OPMODE-ULT-LOWPOWER",  // ENS161 only.
    OPMODE-RESET: "OPMODE-RESET"}

  // REG-REG-CONFIG_: Interrupt Pin Operation.
  static CFG-INT-POL-MASK_   ::= 0b01000000 // RW
  static CFG-INT-DRIVE-MASK_ ::= 0b00100000 // RW
  static CFG-INT-GPRR-MASK_  ::= 0b00010000 // RW Asserts if new data is in GPRR Registers
  static CFG-INT-DAT-MASK_   ::= 0b00000010 // RW Asserts if new data is in DATA-XXX Registers
  static CFG-INT-EN-MASK_    ::= 0b00000001 // Asserts if new data is in DATA-XXX Registers

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

  // REG-STATUS_: data-validity masks:
  static OUTPUT-NORMAL_         ::= 0b00  // OUTPUT is valid
  static OUTPUT-WARM-UP_        ::= 0b01  // OUTPUT is valid but WARMUP (?)
  static OUTPUT-INIT-START-UP_  ::= 0b10  // ENS160 Only.
  static OUTPUT-INVALID_        ::= 0b11

  // REG-DATA-AQI-UBA_ & REG-DATA-AQI-S_: Read masks.
  static AQI-UBA-MASK_ ::= 0b00000111   // Datasheet says 0:2, even if too large.

  // System Timings:  Appear to refer to waits after specific commands:
  static TIMING-RESET_            ::= Duration --ms=50
  static TIMING-STANDARD-MEASURE_ ::= Duration --ms=1000
  static TIMING-CLEAR-GPR_        ::= Duration --ms=2
  static TIMING-TIMEOUT_          ::= Duration --ms=5000

  // MISR: Checksum verification on registers specified in the set $MISR-REGISTERS_.
  // The polynomial used in the CRC computation in REG-DATA-MISR_, 76543210 bit weight factor.
  // 0b00011101 = x^8+x^4+x^3+x^2+x^0 (x^8 is implicit)
  static MISR-POLY_ ::= 0b00011101 // (0x1D)
  static MSIR-IGNORE-REGISTERS ::= {
    REG-PART-ID_,
    REG-DATA-MISR_ }

  // Software tracking of CRC value (updated by $misr-update-software_).
  misr_/int := 0

  // $write-register_ statics for bit width.  All 16 bit read/writes are LE.
  static WIDTH-8_ ::= 8
  static WIDTH-16_ ::= 16
  static DEFAULT-REGISTER-WIDTH_ ::= WIDTH-8_

  static ENS160-HW-ID ::= 0x160
  static ENS161-HW-ID ::= 0x161
  static HW-IDS_ ::= {
    ENS160-HW-ID: "ENS160",
    ENS161-HW-ID: "ENS161"}

  // Class-wide variables:
  hw-id_/int := 0             // Track detected HW version for function use.
  reg_/registers.Registers := ?
  logger_/log.Logger := ?

  constructor
      device/serial.Device
      --startup-operating-mode/int=OPMODE-STANDARD
      --logger/log.Logger=log.default:
    assert: OPMODES_.contains startup-operating-mode
    logger_ = logger.with-name "ens160"
    reg_ = device.registers

    // Check Correct HW ID:
    hw-id_ = get-hardware-id
    if not HW-IDS_.contains hw-id_:
      logger_.error "HW ID unsupported" --tags={"hw-id":"0x$(%02x hw-id_)"}
      throw "Incorrect HW ID"

    // Reset device, returning to OPMODE-IDLE.
    reset OPMODE-IDLE

    // Reset SW MSIR value as device reset will zero the HW value.
    misr-resync_

    // Report device type deteceted and firmware.
    firmware := get-firmware-version
    firmware-string := "v$(firmware[0]).$(firmware[1]).$(firmware[2])"
    logger_.info "$(HW-IDS_[hw-id_]) device started" --tags={"hw-id":"0x$(%02x hw-id_)", "firmware":firmware-string}

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
    logger_.info "currently in operating mode" --tags={"opmode":OPMODES_[get-operating-mode]}

    if is-error:
      logger_.error "currently in ERROR condition"

  /**
  Returns the value of the HARDWARE-ID register.
  */
  get-hardware-id -> int:
    return read-register_ REG-PART-ID_ --width=WIDTH-16_

  /**
  Returns the Firmware version (APPVER).
  */
  /* Function will not answer unless in mode $OPMODE-IDLE. */
  get-firmware-version -> List:
    current-op-mode := get-operating-mode
    out-bytes := [0x00, 0x00, 0x00]

    // Poll to determine if data is ready
    duration := Duration.ZERO
    exception := catch:
      with-timeout TIMING-TIMEOUT_:
        duration = Duration.of:
          if current-op-mode != OPMODE-IDLE: set-operating-mode OPMODE-IDLE
          cmd-no-op_
          cmd-clear-gpr_
          cmd-get-appver_
          while not is-gpr-data-ready:
            sleep --ms=25
          out-bytes[0] = read-register_ (REG-GPR-READ-BASE_ + 4)
          out-bytes[1] = read-register_ (REG-GPR-READ-BASE_ + 5)
          out-bytes[2] = read-register_ (REG-GPR-READ-BASE_ + 6)
          if current-op-mode != OPMODE-IDLE: set-operating-mode current-op-mode

    if exception:
      logger_.error "get-firmware-version - wait for is-gpr-data-ready timed out" --tags={"duration":duration.in-ms}

    return out-bytes

  cmd-get-appver_ -> none:
    write-register_ REG-CMD_ CMD-GET-APPVER_
    // Remove timed wait in favour of checking against $is-gpr-data-ready.
    //sleep TIMING-CLEAR-GPR_

  cmd-clear-gpr_ -> none:
    write-register_ REG-CMD_ CMD-CLR-GPR_
    sleep TIMING-CLEAR-GPR_

  cmd-no-op_ -> none:
    // Don't know why we do this, however is done in ScioSense's examples.
    write-register_ REG-CMD_ CMD-NOP_

  /**
  Resets the device.

  A normal reset would put the device into $OPMODE-DEEPSLEEP.  This function
    defaults to $OPMODE-IDLE unless $mode is optional supplied - if set, a
    the $mode will be set after the reset command is given.
  */
  reset mode/int=OPMODE-IDLE -> none:
    set-operating-mode OPMODE-RESET
    sleep TIMING-RESET_
    misr-resync_
    if mode != OPMODE-DEEPSLEEP: set-operating-mode mode

  /**
  Sets the operating mode.

  Must be one of $OPMODE-DEEPSLEEP, $OPMODE-IDLE, $OPMODE-STANDARD, $OPMODE-RESET.

  In $OPMODE-DEEPSLEEP mode, the ENS160 has limited functionality but will respond to
    a change in mode.  $OPMODE-IDLE is intended for configuration before running an
    active sensing mode.  $OPMODE-STANDARD is the active gas sensing mode.
  */
  set-operating-mode mode/int -> none:
    assert: OPMODES_.contains mode
    current-mode := get-operating-mode

    if current-mode == mode:
      logger_.debug "operating mode already set (doing nothing)" --tags={"opmode":OPMODES_[mode]}
      return

    // Setting OPMODE to IDLE first.  Return if IDLE was the target.
    if current-mode != OPMODE-IDLE and current-mode != OPMODE-RESET:
      write-register_ REG-OPMODE_ OPMODE-IDLE
      sleep TIMING-RESET_
      if mode == OPMODE-IDLE: return

    write-register_ REG-OPMODE_ mode
    if mode == OPMODE-RESET:
      sleep TIMING-RESET_
      return

    // $OPMODE-DEEPSLEEP has $is-opmode-running always returning false, so exit.
    if mode == OPMODE-DEEPSLEEP: return

    duration := Duration.ZERO
    exception := catch:
      with-timeout TIMING-TIMEOUT_:
        duration = Duration.of:
          while not is-opmode-running:
            sleep --ms=100

    if exception:
      logger_.error "set opmode timed out" --tags={"duration":duration.in-ms}
      throw "OPMODE could not be set"
    else:
      logger_.info "set opmode duration" --tags={"mode": OPMODES_[mode],"duration":duration.in-ms}

  /**
  Returns the current operating mode.
  */
  get-operating-mode -> int:
    return read-register_ REG-OPMODE_

  /**
  Set a custom temperature used for calculations.

  This function allows the user to write ambient temperature (in celsius ) to
    the device for compensation. The register can be written at any time.  Set
    to null to remove the configured temperature.
  */
  set-compensation-temp celsius/float? -> none:
    if celsius == null:
      write-register_ REG-TEMP-IN_ 0 --width=WIDTH-16_
      return
    kelvin/float := celsius + 273.15
    raw/int := (kelvin * 64.0).round.to-int
    write-register_ REG-TEMP-IN_ raw --width=WIDTH-16_

  /**
  Get the custom temperature set for calculations.

  See $set-compensation-temp.  Returns null if not set.
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
    if raw == 0: return false
    return true

  /**
  Set the humidity used for calculations.

  This function allows the user to write ambient humidity (in %RH) to
    the device for compensation. The register can be written at any time.  Set
    to null to remove the configured humidity value.
  */
  set-compensation-humidity rh/float? -> none:
    if rh == null:
      write-register_ REG-RH-IN_ 0 --width=WIDTH-16_
      return
    raw := (rh * 512).round.to-int
    write-register_ REG-RH-IN_ raw --width=WIDTH-16_

  /**
  Get the custom humidity used for calculations.

  See $set-compensation-humidity.
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
    if raw == 0: return false
    return true

  /** Whether an OPMODE is running. */
  is-opmode-running -> bool:
    return (read-register_ REG-STATUS_ --mask=STATUS-OPMODE-RUNNING-MASK_) == 1

  /**
  Whether an error is detected.

  E.g. Invalid Operating Mode selected.  The meaning of the errors may be
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
  Whether output data is valid.

  The device needs an initial warm up time from the very first power on.  In
    addition, each time the device is powered on it requires a 3 minute warm up
    period.  These values return which state the device is in.

  Returns one of $OUTPUT-NORMAL_ (normal operation), $OUTPUT-WARM-UP_ (still in
    the 3 minute warm up period), $OUTPUT-INIT-START-UP_ (still in the first
    run, 1 hour initialisation period) and $OUTPUT-INVALID_ (data is invalid).
  */
  data-validity -> int:
    return read-register_ REG-STATUS_ --mask=STATUS-OUTPUT-VALID-MASK_

  /** Shortcut accessor to $data-validity == $OUTPUT-NORMAL_. */
  is-data-valid -> bool:
    return (read-register_ REG-STATUS_ --mask=STATUS-OUTPUT-VALID-MASK_) == OUTPUT-NORMAL_

  /** Returns the Air Quality Index [1..5] as per UBA guidelines. */
  read-aqi-uba -> int:
    return read-register_ REG-DATA-AQI-UBA_ --mask=AQI-UBA-MASK_

  /** Returns the total volatile organic compounds (ppb). */
  read-tvoc -> int:
    return read-register_ REG-DATA-TVOC_ --width=WIDTH-16_

  /** Returns the equivalent CO2 (ppm). */
  read-eco2 -> int:
    return read-register_ REG-DATA-ECO2_ --width=WIDTH-16_

  /** Returns the SocioScense air quality index rate of change. [0-100]. */
  read-aqi-s -> int:
    if not (model-is ENS161-HW-ID):
      logger_.error "aqi-s not available on ENS160"
      return 0
    return read-register_ REG-DATA-AQI-S_ --width=WIDTH-16_

  /**
  Get the temperature used in calculations (degrees celsius).

  Temp is taken from $set-compensation-temp, if supplied.
  */
  get-temp -> float:
    raw := read-register_ REG-DATA-T_ --width=WIDTH-16_
    return (raw.to-float / 64.0) - 273.15

  /**
  Get the humidity used in calculations (%rh).

  Humidity is taken from $set-compensation-humidity, if supplied.
  */
  get-humidity -> float:
    raw := read-register_ REG-DATA-RH_ --width=WIDTH-16_
    return raw.to-float / 512.0

  // Derived measures.

  /** Returns the equivalent ethanol (ppm) value. */
  read-etoh -> int:
    return read-register_ REG-DATA-ETOH_ --width=WIDTH-16_

  /**
  Updates the software instance of the rolling CRC counter.

  The documentation says that the hardware register $REG-DATA-MISR_ is updated
    with every read from a register in the range 0x20 to 0x37, using a CRC
    polynomial (POLY).  In testing it appears that this register is updated for
    every read from the device, regardless of 8 or 16 bit, all except for the
    MISR register itself.  The $read-register_ function has been modified to
    call this function such that for every register read, the function
    $misr-update-software_ is called once for each individual byte read.  This
    keeps the internal variable $misr_ in sync with the hardware register.
    Comparing the Hardware and Software CRC allows one to determine if any data
    reads have become corrupt.
  */
  misr-update-software_ data/int -> none:
    assert: 0 <= data <= 255
    start := misr_
    misr-xor := ((misr_ << 1) ^ data) & 0xFF
    if (misr_ & 0x80) == 0:
      misr_ = misr-xor
    else:
      misr_ = misr-xor ^ MISR-POLY_
    //logger_.debug "misr-update-sw" --tags={"begin":"0x$(%02x start)","data":"0x$(%02x data)","result":"0x$(%02x misr_)"}

  /**
  Returns the hardware's rolling CRC counter.

  Function uses a direct read so as to prevent an 'infinite loop'. (Where the
    MISR register read triggers a MISR register read for comparing the CRC's,
    and so on.)
  */
  misr-hardware_ -> int:
    return (reg_.read-bytes REG-DATA-MISR_ 1)[0]

  /**
  Whether the Hardware MISR and Software MISR are equal - eg, that no
    data read corruptions have occurred.

  When a data read transaction is completed, read $REG-DATA-MISR_, and compare
    it with the software $misr_. They should equal. If not there is a CRC error:
    one or more bytes were corrupted in the transfer.
  */
  misr-valid_ -> bool:
    return misr_ == misr-hardware_

  /**
  Resets the Software MISR value.

  Once the CRC is wrong (or read transactions have been executed without calling
    $misr-update-software_) the software MISR will be out of sync with
    $REG-DATA-MISR_.  Because the CRC uses the previous value in the calculation
    it remains out of sync and every read will appear to be a failure.  This
    function reads REG-DATA-MISR_ and stores in $misr_ to bring the two values
    back in sync.
  */
  misr-resync_ -> none:
    misr_ = misr-hardware_

  model-is model/int -> bool:
    return hw-id_ == model

  /*
  Raw int16 read of the $reg general purpose registers.

  ENS160 datasheet specifies 'Sensor' 1 as R1, and 'Sensor 4' as R4.  The ENS161
    datasheet specifies 'Sensor 4' as R3 - however, the bits and registers are
    the same as R4 in the ENS160 datasheet.  To avoid confusion, input to this
    function is the sensor number instead of the Rx value given in the
    datasheets.
  */
  read-gpr-raw-int16 sensor/int -> int:
    assert: 1 <= sensor <= 4
    if sensor == 2 or sensor == 3:
      logger_.warn "sensor not available according to datasheet" --tags={"sensor":sensor,"hw-id":"0x$(%02x hw-id_)"}
    else if sensor == 1 and model-is ENS161-HW-ID:
      logger_.warn "sensor not available according to datasheet" --tags={"sensor":sensor,"hw-id":"0x$(%02x hw-id_)"}
    reg := sensor - 1
    return read-register_ (REG-GPR-READ-BASE_ + (reg * 2)) --width=16

  /**
  Reads and optionally masks/parses register data.

  Little Endian Only. This version checks and updates software MISR checksum
    value (See $misr-update-software_ toitdocs and MISR information in the
    Datasheet).  Do do this, the *-le functions have been switched to
    .read-bytes, to allow the software CRC to be run against each individual
    byte as required in the datasheet.
  */
  read-register_
      register/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> any:
    assert: (width == 8) or (width == 16)
    raw/ByteArray := #[]

    if mask == null:
      if width == 8: mask = 0xFF
      else: mask = 0xFFFF
    if offset == null:
      offset = mask.count-trailing-zeros

    // Resync - too many registers adjust hardware MSIR value that are not
    // listed in the documentation.  Therefore reset on every 'interesting'
    // read and ignore otherwise:
    //if MISR-REGISTERS_.contains register:
    //if not MSIR-IGNORE-REGISTERS.contains register:
    //  misr-resync_

    register-value/int? := null
    if width == 8:
      raw = reg_.read-bytes register 1
      if signed:
        register-value = LITTLE-ENDIAN.int8 raw 0
      else:
        register-value = LITTLE-ENDIAN.uint8 raw 0
    else:
      raw = reg_.read-bytes register 2
      if signed:
        register-value = LITTLE-ENDIAN.int16 raw 0
      else:
        register-value = LITTLE-ENDIAN.uint16 raw 0

    if register-value == null:
      logger_.error "read-register_ failed" --tags={"register":register}
      throw "read-register_ failed."

    // If the register is not in the set of ignored registers, do MISR SW update:
    if not (MSIR-IGNORE-REGISTERS.contains register):
      //logger_.debug "register included in MISR" --tags={"register":"0x$(%02x register)","size":raw.size,"bytes":raw}
      raw.do: | byte |
        misr-update-software_ byte

      if not misr-valid_: // misr-hw != misr_
        logger_.error "CRC failed" --tags={"hw":"0x$(%02x misr-hardware_)","sw":"0x$(%02x misr_)"}
        misr-resync_
    else:
      // Resync in case the ignored read did change the register.
      if register != REG-DATA-MISR_:
        misr-resync_

    if ((mask == 0xFFFF) or (mask == 0xFF)) and (offset == 0):
      return register-value
    else:
      masked-value := (register-value & mask) >> offset
      return masked-value

  /**
  Writes register data - either masked or full register writes.

  Little Endian Only.  No modifications are required or made to support MISR.
  */
  write-register_
      register/int
      value/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> none:
    assert: (width == 8) or (width == 16)
    if mask == null:
      if width == 8: mask = 0xFF
      else: mask = 0xFFFF
    if offset == null:
      offset = mask.count-trailing-zeros

    field-mask/int := (mask >> offset)
    assert: ((value & ~field-mask) == 0)  // fit check

    // Full-width direct write
    if ((width == 8)  and (mask == 0xFF)  and (offset == 0)) or
      ((width == 16) and (mask == 0xFFFF) and (offset == 0)):
      if width == 8:
        signed ? reg_.write-i8 register (value & 0xFF) : reg_.write-u8 register (value & 0xFF)
      else:
        signed ? reg_.write-i16-le register (value & 0xFFFF) : reg_.write-u16-le register (value & 0xFFFF)
      return

    // Read Reg for modification
    old-value/int? := null
    if width == 8:
      if signed :
        old-value = reg_.read-i8 register
      else:
        old-value = reg_.read-u8 register
    else:
      if signed :
        old-value = reg_.read-i16-le register
      else:
        old-value = reg_.read-u16-le register

    if old-value == null:
      logger_.error "write-register_ read existing value (for modification) failed" --tags={"register":register}
      throw "write-register_ read failed"

    new-value/int := (old-value & ~mask) | ((value & field-mask) << offset)
    if width == 8:
      signed ? reg_.write-i8 register new-value : reg_.write-u8 register new-value
      return
    else:
      signed ? reg_.write-i16-le register new-value : reg_.write-u16-le register new-value
      return
    throw "write-register_: Unhandled Circumstance."

  /**
  Provides strings to display bitmasks nicely when testing.
  */
  bits-grouped_ x/int
      --min-display-bits/int=0
      --group-size/int=4
      --sep/string="."
      -> string:

    assert: x >= 0
    assert: group-size > 0

    // raw binary
    bin := "$(%b x)"

    // choose target width: at least min-display-bits, then round up to a full group
    groups := 0
    leftover := 0
    width := bin.size
    if min-display-bits > width:
      width = min-display-bits
    if group-size > width:
      width = group-size
    leftover = width % group-size
    if leftover > 0:
      width = width + (group-size - leftover)

    // left-pad to target width
    bin = bin.pad --left width '0'

    // group left->right
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
Formats a Duration as HH:MM:SS.mmm, or MM:SS.mmm, or SS.mmm
*/
duration-to-string dur/Duration --width/int=12 -> string:
  total-ms := dur.in-ms
  //print "TOTAL MS: $total-ms"
  sign := ""
  if total-ms < 0:
    sign = "-"
    total-ms = -total-ms

  ms/int := total-ms % 1000
  total-s := (total-ms / 1000).to-int
  //print "$(total-s).$(ms)"

  s/int := total-s % 60
  total-m/int := total-s / 60
  //print "$(%02d total-m):$(%02d s).$(ms)"

  m/int := total-m % 60
  total-h := total-m / 60
  //print "$(%02d total-h):$(%02d m):$(%02d s).$(ms)"

  h/int := total-h % 24
  total-d := total-h / 24

  if total-d > 0:
    return "$sign $(total-d)d $(%02d h):$(%02d m):$(%02d s).$(%03d ms)".pad --left width
  if h > 0:
    return "$sign$(%02d h):$(%02d m):$(%02d s).$(%03d ms)".pad --left width
  else if m > 0:
    return "$sign$(%02d m):$(%02d s).$(%03d ms)".pad --left width
  else:
    return "$sign$(%01d s).$(%03d ms)".pad --left width
