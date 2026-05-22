# Closed-Loop Linear Actuator

> Bachelor project, University of Agder (UiA), 2026
> Custom BLDC motor controller for a leadscrew linear actuator with
> closed-loop position control.

## Overview

This project contains code and production files for a small linear
actuator using the lead screw principle with a custom pcb. The system
consists of a 12 mm BLDC motor with a 1:24 gearbox, integrated hall-
sensors for motor commutation, a AS5600L magnetic encoder module for
position/velocity control. It also supports communication over CAN
and USB-C.

A demonstration video of the actuator can be found here:

[Demonstration video](https://youtube.com/watch?v=XubzCKECgCM)


## Features

- Trapezoidal 3-PWM/6-PWM commutation (Hall-sensor based)
- Closed-loop position control (profiled trajectory + PID)
- Phase current sensing via ADC
- On-chip temperature monitoring with thermal protection
- CAN (FDCAN) communication
- Live telemetry over USB CDC (Teleplotter)

## Hardware

| Component        | Part               |
|------------------|--------------------|
| MCU              | STM32G473RC        |
| Gate driver      | DRV8316 (SPI)      |
| Position encoder | AS5600L (I2C)      |
| Commutation      | Hall sensors       |
| CAN transceiver  | TCAN3403           |
| Motor            | 12 mm BLDC, 1:24   |


## Third Party

The DRV8316 BLDC Driver has been adapted from the[SIMPLEFOC](https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/drivers/drv8316) GitHub library.


## Building & flashing

The CubeIDE project contains custom files for the actuator
within Core/Actuator. The DRV8316 driver, hall encoder and
magnetic encoder is found in Drivers/DRV8316.

The .elf file generated in Debug after building the project
is uploaded to the pcb using STM32CubeProgrammer. First time use
of the pcb requires either a ST-Link debugger using swd or
programming USB-C by setting the mcu in the
DFU(device firmware upgrade) mode.


## Disclaimer

Portions of this firmware were developed with the assistance of
AI-generated code. All such content has been reviewed, tested, and
adapted by the author.

## License
This project is licensed under the MIT License — see the [LICENSE](LICENSE) file.
