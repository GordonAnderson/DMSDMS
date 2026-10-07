# DMSDMS firmware

Firmware for the two modules of a DMSDMS (differential mobility spectrometer) system. The modules are
controlled by a MIPS controller over TWI (I2C) and can also be controlled from a host computer
with ASCII serial commands over USB.

Target: Adafruit Feather M4 Express (SAMD51), built with PlatformIO.

- [Firmware variants](#firmware-variants)
- [Building](#building)
- [How the firmware works](#how-the-firmware-works)
- [CVBIAS firmware](#cvbias-firmware)
- [WAVEFORMS firmware](#waveforms-firmware)
- [Scanning](#scanning)
- [TWI interface](#twi-interface)
- [Serial commands](#serial-commands)
- [Configuration and calibration](#configuration-and-calibration)
- [Source files](#source-files)
- [Project-local copies of framework files](#project-local-copies-of-framework-files)
- [Status](#status)

## Firmware variants

One code base builds two firmware images, selected by the `FIRMWARE` macro set in `platformio.ini`:

| Environment | `FIRMWARE`    | Module |
|-------------|---------------|--------|
| `cvbias`    | 0 (CVBIAS)    | CV and bias voltage control for the two DMS channels, and a two channel electrometer |
| `waveforms` | 1 (WAVEFORMS) | RF waveform generation for the two DMS channels: frequency, duty cycle, drive level and closed loop Vrf control |

Both modules have two channels, numbered 1 and 2 in serial commands and 0 and 1 over TWI.

Do not edit `FIRMWARE` in `DMSDMSMB.h`; it is only a default used when the build does not set it.

## Building

```
pio run -e cvbias -t upload     # build and upload the CVBIAS firmware to the connected board
pio run -e waveforms -t upload  # build and upload the WAVEFORMS firmware
pio run -e cvbias -e waveforms  # build both, no upload
pio run                         # builds only the default environment
```

`default_envs` in `platformio.ini` selects the environment used by a plain `pio run` and by the VS Code Build and
Upload buttons (currently `cvbias`). Change it when you switch to the other module, or pick the environment
in the VS Code status bar. With both boards connected, name the port so the upload goes to the right one:
`pio run -e cvbias -t upload --upload-port /dev/cu.usbmodemXXXX` (`pio device list` shows the ports). Each upload
erases the settings stored in FLASH, so save them again afterward.

Output goes to `.pio/build/<environment>/`. After each build the `.bin` is also copied to the `firmware/` 
folder, named with the firmware and its version, for example `firmware/DAQcvbias_v1.4.bin` and
`firmware/DAQwaveforms_v1.4.bin`. The name and version come from the `Version` string in `src/DMSDMSMB.cpp`
(`"DAQcvbias version 1.4, October 7, 2026"`), so to make a new release change the version there, update the
version history comment above it, and build. A build without a version change replaces the file with the same name.
The copy is done by `copy_firmware.py`, which fails the build if it can't find the version string.

USB stack: the firmware uses the Arduino USB stack, as the Arduino IDE build did. `platformio.ini` sets
`lib_ignore = Adafruit TinyUSB Library` because the Wire, SPI and ZeroDMA libraries include TinyUSB inside 
`#ifdef USE_TINYUSB`, which PlatformIO's library finder does not evaluate. Do not add `-DUSE_TINYUSB`. Libraries
(ArduinoThread, FlashStorage, Adafruit BME280) are fetched by PlatformIO from `lib_deps`.

## How the firmware works

`setup()` reads the two TWI address jumpers, loads the configuration (the `dmsdata` structure) from
FLASH or the built-in defaults, starts the USB serial port and the TWI slave, initializes the
hardware for the variant, and starts a thread.

`loop()` then repeats:

1. Move one received serial character into the receive ring buffer and run any complete command.
2. Run the thread controller. The one thread, `Update`, runs every 25 mS.
3. If MIPS has written a complete configuration through the SEPROM emulation, save it to FLASH.
4. Do any long operation requested by a command (zero the electrometer, set Vrf, build the drive
   table). These are only flagged by the TWI interrupt and are done here.

The `Update` thread reads the monitors and writes the hardware. It keeps `sdata`, the state last
written to the hardware, and only writes an output when the setting in `dmsdata` is different from
`sdata` (or when `sdata.update` forces a full write, for example when a channel is enabled). Commands
therefore only change `dmsdata`, and `Update` applies the change within 25 mS.

Readback values are filtered: a new reading contributes 10% (`FILTER`) to the stored value. A stored value
of -1 means no reading yet and the next reading replaces it.

### Hardware

- **AD5592 analog and digital IO chips** (SPI). They provide the DACs for the bias outputs, reference and
  electrometer offset and zero, and ADC inputs for readback. Chip select pin 4 is the bias chip and
  pin 5 the electrometer chip. Each chip is configured for 0 to 2.5 V (the electrometer chip is set up
  for a 5 V range). Each DAC and ADC channel has a calibration, `counts = value * m + b`.
- **M4 analog inputs.** The electrometer inputs (CVBIAS) are read by the processor's two free running ADCs
  (A0 and A3). The driver monitors (WAVEFORMS) are read with `analogRead`.
- **M4 timers.** TCC3 and TCC1 generate the RF frequency and duty cycle, TCC0 and TCC2 generate the drive
  level PWM (50 kHz), and TC5 generates the scan step interrupt.
- **BME280** environmental sensor (CVBIAS module only), SPI with chip select D9, read with the `RENV` command. It needs
  different SPI settings from the AD5592 chips, which the firmware restores after each read.
- **TWI** is SERCOM2, used as a slave with a modified Wire library (`lib/Wire`).

Pins 40 to 44 (PB10, PB11, PB13, PB14, PA15) are not in the stock Feather M4 pin table. They carry the 
connection between the two processors, the external scan advance input and the rev 2 address line, and are
added by `variants/feather_m4`.

## CVBIAS firmware

Controls the CV (compensation voltage) and bias voltage for each of the two DMS channels.

**CV and bias.** For each channel two outputs, A and B, are generated from the CV and the bias voltage:

```
A = Bias + CV / 2
B = Bias - CV / 2
```

CV and bias are limited to -48 to +48 V. The outputs are DAC channels of the bias AD5592, the actual 
output voltages are read back through its ADC channels (`GCVV`, `GBIASV` return the measured CV and bias).
A disabled channel outputs 0 V on both. A reference voltage (`DCref`, 1.25 V) for the bias amplifiers is 
generated by the electrometer AD5592.

**Electrometer.** A two channel (positive and negative) current electrometer. For each channel there are 
two DACs, an offset and a zero adjustment, and an input. Two ways of reading the input are available:

- the AD5592 ADC, averaged over 10 readings (default), or
- the M4 processor's free running ADC (`M4ena` true), which averages 1024 samples continuously and gives
  an always up to date value.

`ELTMTRZERO` runs the zero procedure: the zero DAC is adjusted, in proportion to the error, until the reading is
inside a small window, for each channel, up to 25 tries.

## WAVEFORMS firmware

Generates the RF waveform for each channel. Each channel has:

| Setting | Meaning | Serial limits |
|---------|---------|---------------|
| Frequency | RF frequency, Hz | 500 kHz to 2 MHz |
| Duty | RF duty cycle, percent | 0 to 90 |
| Drive | Drive FET level, percent of full scale | 0 to 100, also limited by MaxDrive |
| MaxDrive | Largest allowed drive, percent | 0 to 100 |
| MaxPower | Largest allowed power, watts | 10 to 50 |
| Vrf | Peak RF voltage setpoint | 100 to 2000 V |
| Mode | True for closed loop Vrf control | TRUE or FALSE |
| Loop gain | Gain of the Vrf control loop | -10 to 10 |

Readbacks (every 25 mS): the drive supply voltage and current, and the measured Vrf. The power is
`V * I / 1000` and, if it is above MaxPower, the drive is reduced by 1% per update.

**Setting Vrf.** There are three ways:

- **Closed loop** (`SFBMODE,ch,TRUE`): every update the drive is adjusted by `error * gain / 100` toward
  the Vrf setpoint, ignoring errors under 2 V.
- **`SVRF`** (TWI `TWI_SET_VRF_NOW`): starts from the lookup table and then iterates up to 20 times, measuring
  Vrf, until within 1% of the setpoint. Accurate but slow, and it blocks the main loop while it runs.
- **`SVRFF`** (TWI `TWI_SET_VRF_TABLE`): uses only the lookup table. Fast, accuracy depends on the table.

**Drive level to Vrf lookup table.** `CALDRV2VRF` (or TWI `TWI_SET_CAL`) steps the drive from 0 to MaxDrive in
21 points, measures Vrf at each, and stores the 21 values in `LUVrf`. The table must be rebuilt if MaxDrive 
is changed. Scanning uses the table.

## Scanning

A scan steps the CV (CVBIAS) or the Vrf (WAVEFORMS) from a start to an end value in `Steps` equal steps. 
It is advanced either by a timer (TC5, `StepDuration` mS per step) or by a rising edge on an external pin
(`EnableExtStep`, `ExtAdvInput`).

**Which channels are scanned.** There is no channel argument: every channel that has a scan range (start not
equal to end) is scanned, so a scan intended for channel 1 also scans channel 2 if channel 2 has a range set.
The ranges are saved in FLASH, so a range left on a channel from earlier work stays set. A disabled channel is
not scanned (CVBIAS, since version 1.4). To scan one channel only, set the other channel's start and end equal.

At each step the new values are set, the electrometer is read (CVBIAS), and a scan point is reported:

- started by serial (`FBSCNSTRT`), as an ASCII line: `point,time mS,CV1,CV2,positive current,negative current`
- started by TWI (`TWI_SET_STEPSTR`), as a binary `ScanPoint` structure queued for MIPS to read. This can be
  disabled with `TWI_SET_SCNRPT`.

**Number of points.** A scan of `Steps` steps computes `Steps + 1` values (steps 0 through `Steps`) but reports 
`Steps` points. Each timer tick reports the step that was just completed and then applies the next one, and the 
tick that applies the last step stops the scan before reporting. So point *n* holds the values of step
*n* - 1, and the last value is applied to the hardware but not reported. The MIPS software relies on this, 
so it is intentional and should not be changed. The step duration is limited to 5 to 500 mS (the 16 bit
scan timer can count to about 559 mS).

When the scan ends, or is stopped, CV (or Vrf) is restored to the value it had before the scan.
The serial scan commands are only in the CVBIAS firmware.

## TWI interface

The module is a TWI slave. The base address is `0x50` plus `0x02` if address jumper 1 is set and `0x04`
if jumper 2 is set. It responds to three addresses:

| Address | Use |
|---------|-----|
| base | First 256 byte page of the SEPROM emulation |
| base + 1 | Second 256 byte page of the SEPROM emulation |
| base + 0x20 (base + 0x18 if the base has bit 0x20 set) | General commands |

**SEPROM emulation.** MIPS reads and writes the module's configuration structure, `DMSdata`, as it would an
EEPROM: the first byte written is the address within the page, the rest is data. Reads return 32 bytes.
When a write reaches the end of the structure it is saved to FLASH (but not applied until the next restart). 

**General commands.** A write to the command address sends one or more commands: a command byte followed
by its arguments. A command with an invalid channel or an argument outside the limits in `DMSDMSMB.h` is
ignored (there is no error reply over TWI). Channels are 0 or 1 and multi byte values are least significant byte first. Floats are
32 bit IEEE. A read of the command address returns data the module has queued. `TWI_READ_AVALIBLE` returns the 
number of bytes queued, as a 16 bit value, on the next read. `TWI_SET_FLUSH` empties the queue.

| Command | Code | Arguments | Reply |
|---------|------|-----------|-------|
| `TWI_SET_ENABLE` | 0x01 | channel, enable (byte) | |
| `TWI_SET_FREQ` | 0x02 | channel, frequency (int32) | |
| `TWI_SET_DUTY` | 0x03 | channel, duty (byte) | |
| `TWI_SET_MODE` | 0x04 | channel, closed loop (byte) | |
| `TWI_SET_DRIVE` | 0x05 | channel, drive % (float) | |
| `TWI_SET_VRF` | 0x06 | channel, Vrf (float) | |
| `TWI_SET_MAXDRV` | 0x07 | channel, max drive (float) | |
| `TWI_SET_MAXPWR` | 0x08 | channel, max power (float) | |
| `TWI_SET_CV` | 0x09 | channel, CV (float) | |
| `TWI_SET_BIAS` | 0x0A | channel, bias (float) | |
| `TWI_SET_CV_START` / `_END` | 0x0B / 0x0C | channel, CV (float) | |
| `TWI_SET_VRF_START` / `_END` | 0x0D / 0x0E | channel, Vrf (float) | |
| `TWI_SET_DURATION` | 0x0F | step duration mS (int32) | |
| `TWI_SET_STEPS` | 0x10 | number of steps (int32) | |
| `TWI_SET_EXTSTEP` | 0x11 | external step enable (byte) | |
| `TWI_SET_STPPIN` | 0x12 | external step pin (byte) | |
| `TWI_SET_STEPSTR` | 0x13 | none | starts a scan, scan points queued |
| `TWI_SET_STEPSTP` | 0x14 | none | stops a scan |
| `TWI_SET_FLUSH` | 0x15 | none | empties the output queue |
| `TWI_SET_CAL` | 0x16 | channel | builds the drive to Vrf table |
| `TWI_SET_SCNRPT` | 0x17 | report enable (byte) | |
| `TWI_SET_VRF_NOW` | 0x18 | channel, Vrf (float) | sets Vrf with the control loop |
| `TWI_SERIAL` | 0x27 | none | serial command mode, see below |
| `TWI_SET_VRF_TABLE` | 0x29 | channel, Vrf (float) | sets Vrf with the table |
| `TWI_SET_ELEC_POSOFF` / `NEGOFF` | 0x40 / 0x41 | offset (float) | |
| `TWI_SET_ELEC_POSZ` / `NEGZ` | 0x42 / 0x43 | zero (float) | |
| `TWI_SET_ELEC_ZERO` | 0x44 | none | runs the electrometer zero |
| `TWI_SET_ELEC_M4` | 0x45 | M4 ADC enable (byte) | |
| `TWI_READ_READBACKS` | 0x81 | channel | the `ReadBacks` structure |
| `TWI_READ_AVALIBLE` | 0x82 | none | byte count on the next read |
| `TWI_READ_DRIVE` | 0x83 | channel | drive (float) |
| `TWI_READ_VRF` | 0x84 | channel | Vrf setpoint (float) |
| `TWI_READ_CV` | 0x85 | channel | CV setpoint (float) |
| `TWI_READ_ELEC_POS` / `NEG` | 0x86 / 0x87 | none | electrometer current (float) |
| `TWI_READ_ELEC_POSZ` / `NEGZ` | 0x88 / 0x89 | none | electrometer zero voltage (float) |

The `ReadBacks` structure is `CV, Bias` (floats) in the CVBIAS firmware and `V, I, Vrf` (floats) in the
WAVEFORMS firmware. The `ScanPoint` structure is described in `DMSDMSMB.h`.

**Serial over TWI.** `TWI_SERIAL` makes the module treat the bytes it receives as serial commands (the
same commands as USB) and queue the replies for MIPS to read. An ESC (27) byte returns to normal commands.

## Serial commands

Commands are ASCII, sent to the USB serial port (or over TWI, above): the command name, then comma
separated arguments, ended with a newline or semicolon, for example `SFBENA,1,TRUE`. The channel is 1 or 2.

A command is acknowledged with ACK (0x06), or NAK (0x15) with the error code available from `GERR`. A command
that returns a value sends an ACK followed by the value. `MUTE,ON` suppresses all replies and `ECHO,TRUE`
echoes commands. `GCMDS` lists all the commands in the build.

### General (both firmware variants)

| Command | Function |
|---------|----------|
| `GVER` | Report the firmware version |
| `GERR` | Report the last error code |
| `MUTE,ON/OFF` | Turn replies off or on |
| `ECHO,TRUE/FALSE` | Echo mode |
| `DELAY,mS` | Delay (used by macros) |
| `GCMDS` | List the commands |
| `RESET` | Restart the processor |
| `SAVE` / `RESTORE` / `FORMAT` | Save settings to FLASH / restore from FLASH / write the defaults to FLASH |
| `DEBUG` | Debug function |
| `THREADS` / `STHRDENA,name,TRUE/FALSE` | List threads / enable or disable a thread |
| `ARBPGM,addr,size` / `M0PGM,addr,size` | Receive a program file in hex and write it to FLASH |
| `GOTO,addr` / `WHERE` / `ERASEU` | Alternate firmware bank support |
| `SFBENA,ch,TRUE/FALSE` / `GFBENA,ch` | Enable or disable a channel |

### CVBIAS

| Command | Function |
|---------|----------|
| `RENV` | Report BME280 temperature, pressure and humidity (NAK if no sensor was found at startup) |
| `SCV,ch,V` / `GCV,ch` / `GCVV,ch` | Set / get / read back the CV |
| `SBIAS,ch,V` / `GBIAS,ch` / `GBIASV,ch` | Set / get / read back the bias |
| `SFBCVSTRT` / `GFBCVSTRT`, `SFBCVEND` / `GFBCVEND` | Scan start and end CV |
| `SFBSTEPDUR` / `GFBSTEPDUR` | Scan step duration, 5 to 500 mS |
| `SFBNUMSTP` / `GFBNUMSTP` | Number of scan steps, 2 to 10000 |
| `FBSCNSTRT` / `FBSCNSTP` | Start / stop a scan |
| `SFBEXTSTP` / `GFBEXTSTP`, `SFBSTPPIN` / `GFBSTPPIN` | External step advance enable and pin |
| `SELTMTRM4` / `GELTMTRM4` | Use the M4 ADC for the electrometer |
| `GELTMTRPOS`, `GELTMTRNEG` | Electrometer positive and negative current |
| `SELTMTRPOSOFF` / `GELTMTRPOSOFF`, `...NEGOFF` | Electrometer offsets, 0 to 5 V |
| `SELTMTRPOSZERO` / `GELTMTRPOSZERO`, `...NEGZERO` | Electrometer zero voltages, 0 to 5 V |
| `ELTMTRZERO` | Run the electrometer zero procedure |
| `CALDCBREF`, `CALDCBA1`, `CALDCBB1`, `CALDCBA2`, `CALDCBB2` | Calibrate the reference and bias outputs |

### WAVEFORMS

| Command | Function |
|---------|----------|
| `SFBMODE` / `GFBMODE` | Closed loop mode |
| `SFBFREQ` / `GFBFREQ` | Frequency, Hz |
| `SFBDUTY` / `GFBDUTY` | Duty cycle, percent |
| `SFBDRV` / `GFBDRV` | Drive level, percent |
| `GDRVV`, `GDRVI` | Drive supply voltage (V) and current (mA) |
| `SVRF` / `SVRFF` | Set Vrf with the control loop / with the lookup table |
| `GVRF` / `GVRFV` | Vrf setpoint / measured Vrf |
| `GPWR` | Power |
| `SFBMAXDRV` / `GFBMAXDRV`, `SFBMAXPWR` / `GFBMAXPWR` | Drive and power limits |
| `SGAIN` / `GGAIN` | Loop gain |
| `CALVRF` | Calibrate the Vrf readback |
| `CALDRV2VRF` | Build the drive level to Vrf lookup table |
| `CALCUR` | Calibrate the drive current readback |

## Configuration and calibration

All settings and calibrations are in the `DMSdata` structure (`DMSDMSMB.h`). It is loaded from FLASH at 
startup if it holds the valid signature, otherwise the defaults in `DMSDMSMB.cpp` are used. `SAVE`
writes the current settings to FLASH and `FORMAT` writes the defaults. The FLASH area is erased every 
time new firmware is uploaded, so settings and calibrations must be saved again, or sent by MIPS, 
after an upload.

Calibration commands prompt for the value measured with an external instrument, then calculate and apply
the `m` and `b` for the channel:

- Bias outputs (`CALDCB...`): the DAC is set to 0 V and 12 V and the actual voltage is entered each time.
  This calibrates the DAC and the readback ADC.
- `CALVRF`: the drive is set to 10% and 50% and the Vrf measured with a scope is entered.
- `CALCUR`: two drive levels are entered, with the measured current.

Run `SAVE` after calibrating.

## Source files

| File | Contents |
|------|----------|
| `src/DMSDMSMB.cpp` | `setup`/`loop`, the Update threads, TWI handlers, scanning, and the host command functions |
| `src/Serial.cpp` | Receive ring buffer, command table and parser, alternate firmware bank support |
| `src/Hardware.cpp` | Counts conversion, AD5592 and BME280 routines, PWM, FLASH programming, scan timer |
| `src/ADC.cpp` | Free running M4 ADC interrupts (electrometer) |
| `src/Calibration.cpp` | Interactive calibration commands |
| `include/DMSDMSMB.h` | Configuration structures, TWI command codes, pin and channel assignments |
| `include/Hardware.h`, `ADC.h`, `Serial.h`, `Calibration.h`, `Errors.h` | Declarations for the files above |
| `include/AtomicBlock.h` | Interrupt blocking helper (third party) |
| `copy_firmware.py` | Post build script that copies the `.bin` to `firmware/` with the version in the name |
| `firmware/` | Released `.bin` files |
| `todo.md` | Open issues from the code review |
| `TESTING.md` | Bench test plan with a results record |

## Project-local copies of framework files

These folders look like copies of upstream code, and they are. They are needed. Do not delete them, and
do not replace them with the stock versions.

- `variants/feather_m4/` - the Adafruit Feather M4 variant with five extra pins added (40..44 =
  PB10, PB11, PB13, PB14, PA15). The stock variant only defines pins 0..39, and this firmware uses the
  extra pins for the TWI address line, the link between the two processors, and the external scan
  advance input. `platformio.ini` selects it with `board_build.variants_dir`. Changes from stock are
  marked in `variant.cpp` and `variant.h` (`PINS_COUNT` is 45).
- `lib/Wire/` - the Wire library with `onAddressMatch()` added, which is used to tell which of the module's
  TWI addresses a transfer was sent to. Stock Wire does not have it.
- `lib/SerialBuffer/` - output buffer used for replies sent back over TWI.

If the Adafruit core is updated, re-check these against the new upstream versions.

## Status

This firmware was moved from the Arduino IDE to PlatformIO and has not yet been tested on hardware in 
this form. A code review found a number of open issues, listed in `todo.md`. Notable ones: scan advance
and some TWI command handling run in interrupt context, and the scan timer cannot do step durations over about 559 mS.
