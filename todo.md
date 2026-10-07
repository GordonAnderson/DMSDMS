# DMSDMS code review - to do

Found by code review of the PlatformIO port. Nothing here has been tested on hardware.
Line numbers are from the time of the review and will drift as the code changes.

## High

- [x] **Pins 40-44 missing from the stock core.** Fixed with the project-local `variants/feather_m4/` (see README).
- [x] **`RENV` / BME280 pin clash.** The BME280 is only on the CVBIAS module and uses D9 as its chip select. It is now compiled only into the CVBIAS firmware, is initialised in `setup()`, and the SPI settings are restored after each read. Still to check on hardware: that `RENV` reports sensible values and that the bias outputs are unaffected afterward.
- [x] **Out-of-range channel index over TWI.** No `b` is 0 or 1 check in `receiveEventProcessor` (`DMSDMSMB.cpp`) for `TWI_SET_VRF_START`, `TWI_SET_VRF_END`, `TWI_READ_DRIVE`, `TWI_READ_VRF`, `TWI_SET_CV_START`, `TWI_SET_CV_END`, `TWI_READ_CV`. A bad byte reads or writes outside `channel[]`.
- [x] **Uninitialised value in `SVRF` / `SVRFF`.** `setVrfCmd` and `setVrfTableCmd` (`DMSDMSMB.cpp:1359`, `:1368`) ignore the result of `setVariable` and call `SetVrf` / `SetVrfTable` with an uninitialised `fval` after sending a NAK.

## Medium

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

## Housekeeping

- [ ] Delete `src/build/` (old Arduino IDE build output) or add it to `.gitignore`.
- [ ] Commit `variants/` and `lib/` with the project (see README).
