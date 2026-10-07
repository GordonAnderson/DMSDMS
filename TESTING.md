# DMSDMS firmware test plan

Bench test of the PlatformIO port (version 1.3) against the expected behavior of the Arduino firmware. 
Test one board at a time. Copy the record block below for each run and tick the items as you go.

The firmware has not been run on hardware since the port. The items are ordered by risk: first what the 
port could have broken (pins, TWI, flash), then each module, then MIPS integration.

```
Board:            CVBIAS / WAVEFORMS
Firmware file:    firmware/DAQ______v1.3.bin
Board serial no.:
Tester / date:
TWI address jumpers:
Notes:
```

For any failure record the exact command sent, what the board did and the serial output. Note anything that 
looks different from the old Arduino firmware, even if it still works.

## 0. Before you start

- [ ] If the board holds settings or calibrations you need, read them off first (calibration values, or the
      SEPROM contents over TWI). Uploading new firmware erases the settings stored in FLASH.
- [ ] Keep the old Arduino firmware image (`.hex`/`.bin`) handy so you can reflash it and compare behavior.
- [ ] USB terminal at 115200 on the board's serial port. Commands end with a newline, for example `GVER`.
- [ ] Upload with one board connected: `pio run -e cvbias -t upload` or `pio run -e waveforms -t upload`
      (see the README for naming the port if both are connected).

## 1. Both variants: what the port could have broken

- [ ] **1.1 Boot and identity.** `GVER` returns `DAQcvbias version 1.3, October 6, 2026` or
      `DAQwaveforms version 1.3, October 6, 2026`. `pio device list` (or macOS System Information) shows the 
      product name `CVBIAS` or `WAVEFORMS` and the manufacturer `GAA Custom Electronics, LLC`.
- [ ] **1.2 Command list.** `GCMDS` lists the commands for this variant only (`SCV` etc. on CVBIAS, `SFBFREQ` etc.
      on WAVEFORMS, `RENV` on CVBIAS only).
- [ ] **1.3 Pin table, TWI address (highest risk).** For each combination of the two address jumpers read the 
      module's TWI address (MIPS, or a bus scanner). Expected base address `0x50`, plus `0x02` for jumper 1 and
      `0x04` for jumper 2. The module also answers at base + 1 and at the command address (`0x70` for base `0x50`).
      A wrong or missing address points to the extra pins (40 to 44) not working.
- [ ] **1.4 Error replies.** `SFBENA,3,TRUE` and `SFBENA,x,TRUE` both return NAK (0x15), `GERR` reports error 2, 
      and the channel state is unchanged. `SFBENA,1,TRUE` then `GFBENA,1` returns `TRUE`.
- [ ] **1.5 Settings in FLASH.** `FORMAT`, `RESET`. Change a value, `SAVE`, `RESET`, and confirm the value survived.
      Change another value, `RESTORE`, and confirm it returns to the saved one.
- [ ] **1.6 TWI SEPROM.** From MIPS read the 512 byte configuration structure and compare with the saved 
      settings. Write it back and confirm it is saved to FLASH (after `RESET` the settings are loaded).
- [ ] **1.7 Update thread.** `THREADS` lists `Update` with a 25 mS interval.
- [ ] **1.8 Serial over TWI.** Send `TWI_SERIAL` (0x27) from MIPS, then a command such as `GVER`, and read the reply
      back. Send ESC (27) and confirm the module returns to normal TWI commands.
- [ ] **1.9 Mute and echo.** `MUTE,ON` suppresses replies, `MUTE,OFF` restores them. `ECHO,TRUE` echoes commands.

## 2. CVBIAS module

Use a voltmeter on the bias outputs. Channels are 1 and 2.

- [ ] **2.1 Bias outputs.** `SFBENA,1,TRUE`. For several values (0, +1, -1, +10, -10, +40, -40) set
      `SCV,1,<cv>` and `SBIAS,1,<bias>` and measure outputs A and B. Expected: `A = bias + CV/2`,
      `B = bias - CV/2`. Repeat for channel 2.
- [ ] **2.2 Disable.** `SFBENA,1,FALSE` returns both outputs of the channel to 0 V.
- [ ] **2.3 Limits.** `SCV,1,49` and `SBIAS,1,-49` return NAK (limits are -48 to +48).
- [ ] **2.4 Readbacks.** `GCVV,1` and `GBIASV,1` agree with the voltmeter at several settings (this also confirms
      the SPI mode and the ADC channel assignments).
- [ ] **2.5 Reference.** The bias amplifier reference (default 1.25 V) is correct. `CALDCBREF` runs and the 
      prompts accept a measured value.
- [ ] **2.6 Bias calibration.** `CALDCBA1`, `CALDCBB1`, `CALDCBA2`, `CALDCBB2` run. Compare the resulting 
      `m` and `b` with the old firmware's values. Run `SAVE` afterward.
- [ ] **2.7 Electrometer zero.** With no input `ELTMTRZERO` completes and `GELTMTRPOS` / `GELTMTRNEG` are 
      near zero.
- [ ] **2.8 Electrometer readings.** With a known input compare the AD5592 reading with `SELTMTRM4,TRUE` (M4 ADC
      reading). Both channels, positive and negative.
- [ ] **2.9 Electrometer offsets.** `SELTMTRPOSOFF` and the zero commands accept 0 to 5 and return NAK
      outside the range.
- [ ] **2.10 BME280.** `RENV` reports plausible temperature, pressure, altitude and humidity.
- [ ] **2.11 SPI recovery after RENV (important).** Immediately after `RENV` re-measure the bias outputs and 
      readbacks (2.1 and 2.4). They must be unchanged. If they are not, the SPI settings were not restored.
- [ ] **2.12 No sensor.** On a board without the BME280 (or with it disconnected), `RENV` returns NAK and
      `GERR` reports 23.

## 3. WAVEFORMS module

RF voltage is present. Start with the load disconnected or the drive supply low, use a scope on the RF output
and keep the drive at 0 until you are ready. Defaults: 1.2 MHz, duty 52, drive 0, max power 25 W, max drive 50%.

- [ ] **3.1 Idle.** With drive 0 the outputs are idle on both channels.
- [ ] **3.2 Frequency and duty.** `SFBFREQ`, `SFBDUTY` for both channels and check on the scope (try 500 kHz, 
      1.2 MHz, 2 MHz, duty 10, 52, 90). `SFBFREQ,1,400000` and `SFBDUTY,1,91` return NAK.
- [ ] **3.3 Drive level.** Raise `SFBDRV` in small steps (1, 5, 10, 20%). The drive supply readbacks `GDRVV` 
      and `GDRVI` agree with a meter. Drive above `SFBMAXDRV` is limited.
- [ ] **3.4 Disable.** `SFBENA,1,FALSE` brings the drive to 0.
- [ ] **3.5 Vrf readback calibration.** `CALVRF,1` (and 2) prompts and calculates `m` and `b`. Compare with the
      old firmware's values. Run `SAVE` afterward. (`CALCUR` for the current readback if needed.)
- [ ] **3.6 Drive to Vrf table.** `CALDRV2VRF,1` completes (about 5 s, no serial response during it) and the 
      table values increase with drive.
- [ ] **3.7 Setting Vrf.** `SVRF,1,<volts>` and `SVRFF,1,<volts>` at three setpoints. `GVRFV` and the scope agree.
      `SVRF,1,50` returns NAK and does nothing (minimum 100 V).
- [ ] **3.8 Power limit.** Set `SFBMAXPWR` low and confirm the drive backs off when the power exceeds it.
- [ ] **3.9 Closed loop.** `SFBMODE,1,TRUE`, change the Vrf setpoint, watch it settle and stay stable. Try
      different `SGAIN` values.

## 4. Scanning

CVBIAS first (serial scan commands exist only there).

- [ ] **4.1 Serial scan.** Set `SFBCVSTRT`, `SFBCVEND`, `SFBNUMSTP` (for example 10) and `SFBSTEPDUR` (for example
      100), then `FBSCNSTRT`. Expected: lines `point,time,CV1,CV2,positive,negative` and **exactly `Steps` points**
      (the last value is applied but not reported, this is intentional). CV returns to its original value 
      afterward.
- [ ] **4.2 Limits.** `SFBSTEPDUR,5` is accepted, `SFBSTEPDUR,501` and `SFBNUMSTP,1` return NAK. Run a scan at 5 ms
      and at 500 ms and compare the time stamps with the step duration.
- [ ] **4.3 Stop.** `FBSCNSTP` during a scan stops it and restores the CV.
- [ ] **4.4 External step advance.** `SFBEXTSTP,TRUE` and `SFBSTPPIN,<pin>`, then advance with a rising edge on the
      pin used in the system. This depends on the extra pins in the pin table.
- [ ] **4.5 TWI scan.** From MIPS start a scan with `TWI_SET_STEPSTR`, read back the scan points and compare with 
      the serial scan. `TWI_SET_STEPSTP` stops it mid-scan and the values restore.
- [ ] **4.6 WAVEFORMS scan (TWI).** Same for a Vrf scan (`TWI_SET_VRF_START` / `_END`).

## 5. TWI input checks (test program, not the real MIPS code)

- [ ] **5.1 Bad channel.** Send each channel command with channel byte 2 and 0x80 (for example `TWI_READ_DRIVE`, 
      `TWI_SET_CV_START`). The module ignores them, returns no data and keeps running (check `GVER` afterward).
- [ ] **5.2 Out of range values.** Send frequency 0, duty 200, steps 0, step duration 0 and 2000, and a CV of 100. 
      None take effect and the module does not hang or reset.

## 6. MIPS integration

- [ ] **6.1** Run the real MIPS acquisition that uses the module and compare the results with the old firmware.
- [ ] **6.2** Leave the system running for an extended period (an hour) with the Update thread active and 
      confirm readbacks stay stable and nothing resets.

## Things to watch for

These are known open items from the code review (see `todo.md`). Note whether you see them:

- Serial characters can be dropped during long operations (`SVRF`, calibrations, `ELTMTRZERO`) because the main
  loop is busy. Send commands slowly or wait for the reply.
- The scan timer, and some TWI commands, do their work in interrupt context. Look for glitches in the bias or 
  RF outputs while a scan starts or stops, or while MIPS is sending commands.
- The alternate firmware bank commands (`GOTO`, `ERASEU`) use an M0 address and have not been checked. Do not 
  use them on the first bench test.

## Results summary

| Board | Version | Date | Passed | Failed | Notes |
|-------|---------|------|--------|--------|-------|
|       |         |      |        |        |       |
