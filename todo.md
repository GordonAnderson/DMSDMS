# DMSDMS code review - to do

Found by code review of the PlatformIO port. Nothing here has been tested on hardware.
Line numbers are from the time of the review and will drift as the code changes.

## High

- [x] **Pins 40-44 missing from the stock core.** Fixed with the project-local `variants/feather_m4/` (see README).
- [x] **`RENV` / BME280 pin clash.** The BME280 is only on the CVBIAS module and uses D9 as its chip select. It is now compiled only into the CVBIAS firmware, is initialised in `setup()`, and the SPI settings are restored after each read. Still to check on hardware: that `RENV` reports sensible values and that the bias outputs are unaffected afterward.
- [x] **Out-of-range channel index over TWI.** No `b` is 0 or 1 check in `receiveEventProcessor` (`DMSDMSMB.cpp`) for `TWI_SET_VRF_START`, `TWI_SET_VRF_END`, `TWI_READ_DRIVE`, `TWI_READ_VRF`, `TWI_SET_CV_START`, `TWI_SET_CV_END`, `TWI_READ_CV`. A bad byte reads or writes outside `channel[]`.
- [x] **Uninitialised value in `SVRF` / `SVRFF`.** `setVrfCmd` and `setVrfTableCmd` (`DMSDMSMB.cpp:1359`, `:1368`) ignore the result of `setVariable` and call `SetVrf` / `SetVrfTable` with an uninitialised `fval` after sending a NAK.

## Medium

- [ ] **Scan drives a disabled channel (found from bench report: noise on channel 2 while scanning channel 1).** `SetScanParameters` scans every channel whose CV start and end differ, without checking `Enable`. For a disabled channel `Update` writes 0 V every 25 ms, so the outputs alternate between the scan value and 0 V. Fixed in 1.4: disabled channels are skipped. Still to confirm on hardware, see the checks below. Note the start/end range is saved in FLASH, so a range left on channel 2 from earlier work stays set.

- [x] **Scan step duration over about 559 ms.** Limited to 500 ms (`MAXSTEPDUR`), which is enough for the application. Note a value over 500 already saved in FLASH from older firmware is not corrected when loaded.
- [x] **Last scan point never reported.** Not a bug: the MIPS software depends on this behavior. Left unchanged and documented in `DMSDMSMB.cpp` and the README.
- [x] **No TWI command to stop a scan.** `TWI_SET_STEPSTP` (0x14) is defined in `DMSDMSMB.h` but has no `case` in `receiveEventProcessor`.
- [x] **Validate TWI values like the serial path does.**
  - `TWI_SET_FREQ`: a value of 0 divides by zero in `setFreqDuty`.
  - `TWI_SET_DUTY`: values above 100 give a bad PWM compare value.
  - `TWI_SET_STEPS`: a value of 0 divides by zero in `SetScanParameters`.
  - `TWI_SET_DURATION`: 0 or negative gives a timer count of 0 and an interrupt storm.
  - `TWI_SET_STPPIN` and `TWI_SET_VRF`: no limits.
- [ ] **Heavy work in interrupt context.**
  - `ScanISR` runs from the TC5 interrupt (priority 0) and does SPI transfers, PWM updates and `sb.write`.
  - `TWI_SET_STEPSTR` runs `InitScan` inside the I2C interrupt, which reconfigures timers and attaches interrupts.
  - `sb` is written from an interrupt and read from the Wire request handler with no protection.

  Set flags in the interrupts and do the work from the main loop where possible.
- [ ] **Serial bytes can be dropped.** `ProcessSerial` (`DMSDMSMB.cpp:899`) takes one character per call. `SetVrf` (about 0.6 s), `CalibrateVrf2Drive` (about 5 s) and `ZeroElectrometer` block the loop, so the USB buffer can overflow, and `RB_Put` silently discards when full. Change the `if` to a `while`.
- [ ] **`EraseUpper` / `ProgramGOTO` use the M0 address.** They jump to `0x2000`, but the Feather M4 bootloader is 16 KB and the application starts at `0x4000` (`Serial.cpp:177-181`). Only matters if those commands are used.
- [x] **`UpdateADCvalue` reads the AD5592 twice.** The second read (`DMSDMSMB.cpp:571`) isn't checked for -1, so a failed read feeds garbage into the filter.

## Low

- [x] **Unchecked `sscanf`.** `checkChannel(char*)` and `setVariable(int*)` use the result without checking it, so a bad argument leaves the variable uninitialised.
- [ ] **`calCurrent` ends with `Drive = 10`** (`Calibration.cpp:232`). It looks like it meant to restore the original drive. It also never clamps the drive entered by the user.
- [ ] **`StopScan` uses the current `EnableExtStep`.** If the flag changed mid-scan, it detaches or resets the wrong source.
- [ ] **`SaveSettings` doesn't refresh `Ebuf`.** MIPS can read stale SEPROM data until the next reset.
- [ ] **ADC window flag never cleared.** In `ADC.cpp` the `WINMON` flag is never cleared, and `MAXADC` is 4095 while results are 16-bit. Only matters if `ADCmode` is nonzero, which nothing sets today.
- [ ] **Electrometer channel comments disagree.** The comments say CH4 is positive, but `POSELEC` is 5 and `NEGELEC` is 4. Check the wiring and fix whichever is wrong.
- [ ] **Check `sizeof(DMSdata)` fits the 512-byte `Ebuf`** in both variants. A `static_assert` would catch this at build time.

## Second review (October 7, 2026)

- [x] **USB stack changed by the port (fixed in 1.4).** The original Arduino build used the Arduino USB 
  stack. The port added `-DUSE_TINYUSB`, which switched to TinyUSB. Now `lib_ignore = Adafruit TinyUSB Library` and the 
  Arduino stack is used again. The v1.3 `.bin` files were built with TinyUSB, use v1.4.
- [ ] **Power limit not enforced during long Vrf operations** (WAVEFORMS). `SVRF` / `TWI_SET_VRF_NOW`, `CALDRV2VRF` / 
  `TWI_SET_CAL` and `CALVRF` run in the main loop for up to several seconds, and the Update thread (which reduces 
  the drive above MaxPower) does not run meanwhile. `SetVrf` can raise the drive to MaxDrive, and `CalibrateVrf2Drive`
  steps it to MaxDrive. Only MaxDrive limits the drive during these operations.
- [ ] **Restarting a scan loses the original CV.** `InitScan` called while a scan is running (`TWI_SET_STEPSTR` or 
  `FBSCNSTRT` twice) saves the mid-scan CV as the value to restore, so the CV is left at the wrong value after the 
  scan. Fix: stop a running scan first.
- [ ] **`StopScan` restores the CV of both channels**, including a channel that was not scanned. A CV change made on
  the other channel during a scan is undone when the scan ends. Fix: restore only the channels that were scanned.
- [ ] **Serial over TWI receive buffer race.** In TWI serial mode `PutCh` runs in the I2C interrupt while the main 
  loop takes characters with `RB_Get`. `Count` and `Commands` are updated without blocking interrupts, so characters
  or command counts can be lost. Fix: block interrupts around the updates in `RB_Put`/`RB_Get`.
- [ ] **Bias output after calibration** (CVBIAS). `CALDCBA1`..`CALDCBB2` leave the output at +/-CV/2 without the bias, 
  and `Update` does not rewrite it with the new calibration until the CV or bias is changed. Fix: set
  `sdata.update = true` at the end of the calibration.
- [ ] **RF frequency is low by one timer count** (WAVEFORMS). `setFreqDuty` sets `PER = MCK / freq`, but the period is
  `PER + 1` counts: 1.2 MHz gives 1.188 MHz (-1%), 2 MHz gives 1.967 MHz (-1.6%). Same in the original firmware. Check
  with a scope (TESTING.md 3.2) before changing, existing calibrations may assume it.
- [ ] **Vrf calibrations need the channel enabled with drive above 0** (WAVEFORMS). With drive 0, Update has set the RF
  duty to 0, so `CALDRV2VRF` / `TWI_SET_CAL` measure with no RF. On a disabled channel Update forces the drive to 0
  every 25 ms during `CALVRF`. Only a code comment warns about this. Fix: check and NAK, or turn the RF on first.
- [ ] **A serial command with a missing argument also NAKs the next command.** For example `SCV,1` followed by
  `GCV,1`: the parser takes the newline as the missing argument, then rejects `GCV` and `1`. Probably the same in 
  MIPS, low priority.
- [ ] **SEPROM save triggers one byte early.** `receiveEvent` compares `Eaddress + howMany` (which counts the address
  byte) with `sizeof(DMSdata)`. Harmless if MIPS writes the last byte in the same transfer.
- [ ] **TWI address mask also answers base + 0x21** (for example 0x71), which is treated as a SEPROM page 1 write.
  Check it does not collide with another device on the MIPS bus.

## Housekeeping

- [ ] Delete `src/build/` (old Arduino IDE build output) or add it to `.gitignore`.
- [ ] Commit `variants/` and `lib/` with the project (see README).
