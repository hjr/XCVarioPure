# XCVario Pro

An ESP32-based, lean variometer system with an open data interface supporting OpenVario, Cambridge, Borgelt, Naviter, and XCVario formats, running on dedicated XCVario hardware.

![Vario](https://github.com/hjr/XCVarioPro/blob/master/XcvPro.png)

The project is rooted in the open-source and open-hardware flight computer community and is based on the ESP32 SoC, supporting modern digital sensors such as BMP280, SPL06-007, DS1820B, TE4525, ABPMRR, MP5004DP, AHRS MPU6050, and the QMC5883L magnetometer.

The variometer features a sunlight-readable 2.4-inch IPS LCD display with a brightness of 1000 nits and a low power consumption of less than 1.2 W, corresponding to approximately 100 mA at an operating voltage of 12.5 V. The device measures 64 (H) × 64 (W) × 35 (D) mm, fitting a standard 57 mm (2″) instrument gauge, and weighs approximately 170 grams.

## Vision
While the amazing availability of high-fidelity sensors combined with today’s highly miniaturized computational power provides the perfect ingredients for a project like this, it quickly becomes clear that the overall quality of the resulting system design is tightly linked to the quality of the software it is built on. Recognizing this, it also becomes evident that further improvements in quality and usability of a variometer like this are hardly achievable with a “pick-it-for-free” mentality. Significant effort is required, and that effort needs to be funded — which in turn becomes a major challenge for the project.

The evolution from **XCVario** to **XCVario Pro** therefore comes with the addition of a dedicated software expert to the XCVario team, responsible for all software-related aspects. The variometer will retain a basic feature set based on an open-source code base, as it did initially. In addition, it will offer licensed features intended to fund ongoing maintenance and further development, with the goal of evolving the variometer into a best-in-class product.

The original ambition of providing a high-quality variometer sensor that turns any club glider with an outdated variometer into a fun-to-fly aircraft remains unchanged. Beyond this, several new capabilities will be introduced, addressing key limitations found in common variometer designs that currently prevent optimal performance, along with a number of new features:

* Creation of a synchronized sensor data history for future filter design
* Easy-to-perform high-precision IMU reference calibration
* Continuous gyro drift tracking
* “imuTEK” total energy compensation of the vario signal, with awareness of horizontal wind gusts
  - automated and continuous calibration of the compensation
  - no requirement for a TE probe
* Instant wind calculation

## Images and video material

[More recent materials on the XCVario face-book page](https://www.facebook.com/p/XCvario-100090831756791/)

[A lot of history to the project on YouTube](https://www.youtube.com/results?search_query=xcvario)

## Supported sensors
* TE variometer (TE)
* Airspeed (PI)
* Barometric altitude (BA)
* Outside air temperature
* Battery voltage
* Acceleration and rotation IMU
* Optional earth magnetic field sensor
* Optional flap sensor
* Optional gear warn sensor
* Optional GPS NMEA data stream
* Optional Flarm traffic NMEA data stream
* Optional external instant wind sensors (Anemoi)

The ESP32 module contains a Bluetooth and Wi-Fi  module plus serial interfaces, so we are able to transmit wireless or wired data to any navigation device running XCSoar, LK8000, Naviter and more in various formats as there is the native XCVario, Borgelt, Cambridge and Openvario format,so devices can operate as full glide computer with TE-vario, barometric altitude, speed and more.

## The manual

[Online Handbook in various languages](https://xcvario.com/docs-category/vario)

## A (soft-)feature list
* Variometer display with adjustable range (1 m/s – 30 m/s) and damping (1 s – 10 s)
* QNH, ballast, bugs, and McCready adjustment, plus many other configurable options
* Vario sound generator with adjustable volume and deadband, plus setup options for tone style profiles
* Sound sequencer with a variety of sound bites for sonification of most visual variometer information
* Integrated loudspeaker with 2-watt audio power
* S2F (Speed-to-Fly) indicator with configurable MC, ballast, and bugs based on glider polars
* Around 150 predefined polars included (list: https://github.com/hjr/XCVarioPro/blob/master/components/glider/PolarTable.txt)
* IAS or TAS airspeed indication
* OAT (Outside Air Temperature) sensor
* Flap assistant with or without flap position sensor
* Battery voltage and status indicator
* Thermal assistant indicator for efficient thermal centering
* Wind rose indicator with optional second, instant wind pointer
* Bluetooth, Wi-Fi, and two serial interfaces for external devices (XCSoar, LK8000, FLARM, Anemoi, other sensors …)
* Complete support for a secondary variometer (two-seater) with automatic routing to all connected devices
* High-precision barometric altimeter with 1 hPa (≈ 8 m) absolute accuracy and 0.1 m resolution
* Sunlight-readable, high-contrast 2.4-inch IPS display with 1000 nits brightness
* External switch for S2F / Vario mode
* Setup menu for customization of variometer features
* Connectivity for the FLARM View display
* Output of NMEA data in OpenVario, Borgelt, Cambridge, Naviter, or XCVario format
* OTA (Over-the-Air) software updates via Wi-Fi and web browser
* Attitude and Heading Reference System (AHRS) available since the 2021 hardware revision
* Wind calculation in circling and straight flight (requires a connected GPS source)
* Electronic TE compensation, eliminating long TE tubing and TE probe limitations