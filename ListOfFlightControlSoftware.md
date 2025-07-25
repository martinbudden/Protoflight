# Flight control software

This is a partial list of flight control software.

Descriptions are taken from the software's own web/github pages

## Historical

### [multiwii](https://github.com/multiwii/multiwii-firmware)

8-bit flight controller firmware for Arduino.

### [baseflight](https://github.com/multiwii/baseflight)

32-bit fork of the MultiWii RC flight controller firmware

### [cleanflight](https://github.com/cleanflight/cleanflight)

Clean-code version of the baseflight flight controller firmware.

Cleanflight is forked from Baseflight, Cleanflight was forked by Betaflight, Cleanflight is again forked from Betaflight.

Cleanflight 4.x -> betaflight 4.x -> Cleanflight v2.x -> Betaflight 3.x -> Cleanflight v1.x -> Baseflight -> MultiWii

## The big three

### [betaflight](https://betaflight.com/)

Pushing the Limits of UAV Performance
Betaflight is the world's leading multi-rotor flight control software.
The global FPV drone racing and freestyle community choose Betaflight for its performance, precision, cutting edge features, reliability and hardware support.
[source code](https://github.com/betaflight/betaflight)

### [iNav](https://github.com/iNavFlight/inav/wiki)

INAV is a Free and Open Source Flight Controller and Autopilot Software System that is actively developed with large releases annually, and support releases as needed.
[source code](https://github.com/iNavFlight/inav)

### [ardupilot](https://ardupilot.org/)

ArduPilot is a trusted, versatile, and open source autopilot system supporting many vehicle types:
multi-copters, traditional helicopters, fixed wing aircraft, boats, submarines, rovers and more.
The source code is developed by a large community of professionals and enthusiasts.
[source code](https://github.com/ArduPilot/ardupilot)

## Others

The following flight control software is more focused on learning, education, research, and experimentation rather than features or performance.

### [drehmflight](https://www.drehmflight.com/)

dRehmFlight is the flight controller for hobbyists, hackers, and non-coders interested in stabilizing their wacky and unique flying creations.
The code and supporting documentation is built to bring someone up to speed on VTOL flight stabilization concepts as quickly and painlessly as possible.
The code is written and presented in a way that is intuitive, easy to follow, and most importantly: all in one place.
[source code](https://github.com/nickrehm/dRehmFlight)

### [madflight](https://madflight.com/)

madflight is a toolbox to build high performance flight controllers with Arduino IDE or PlatformIO for ESP32-S3 / ESP32 / RP2350 / RP2040 / STM32. Inspired by dRehmFlight.
[source code](https://github.com/qqqlab/madflight)

### [BitCraze Crazyflie](https://www.bitcraze.io/)

Crazyflie is a small, versatile quadcopter for research and education.
It has an ecosystem of expansion decks, clients, and development tools.
[source code](https://github.com/bitcraze/crazyflie-firmware)


### [ESP-Drone](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/gettingstarted.html)

ESP-Drone is an ESP32/ESP32-S2/ESP32-S3 based flying development board provided by Espressif.
ESP-Drone is equipped with Wi-Fi key features, which allows this drone to be connected to and controlled by an APP or a gamepad over a Wi-Fi network.
This drone comes with simple-structured hardware, clear codes, and supports functional extension.
Therefore, ESP-Drone can be used in STEAM education. Part of the code is from Crazyflie open source project under GPL3.0 license.

### [M5Stack StampFly](https://docs.m5stack.com/en/app/Stamp%20Fly)

StampFly is an open source educational drone developed by Kouhei Ito and M5Stack.
See [Stamp Fly: An Open-Source DIY Drone Kit from Japan-Shenzhen](https://www.hackster.io/stampfly/stamp-fly-an-open-source-diy-drone-kit-from-japan-shenzhen-93d099).

It is a quadcopter based on the ESP32-S3, and it was created it with the goal of making it "legally flyable in Japan and usable for drone control classes"
[source code](https://github.com/m5stack/M5StampFly)

M5Stack StampFly is based on [Kanazawa StampFly](https://github.com/M5Fly-kanazawa/StampFly2024June)

Kouhei Ito is a Professor at Kanazawa Institute of Technology and has lots of great articles (in Japanese) about drones and the related mathematics on his [blog](https://rikei-tawamure.com/).
Google Translate makes these articles accessible if you don't speak Japanese.

### [Reefwing](https://github.com/Reefwing-Software)

not a flight controller as such, but a collection of libraries that support flight control software.
Including

1. [Reefwing-AHRS](https://github.com/Reefwing-Software/Reefwing-AHRS) Attitude and Heading Reference System
2. [Reefwing-MSP](https://github.com/Reefwing-Software/Reefwing-MSP) Reefwing MultiWii Serial Protocol
3. [Reefwing-SBUS](https://github.com/Reefwing-Software/Reefwing-SBUS) SBUS Library for the Nano 33 BLE

and others
