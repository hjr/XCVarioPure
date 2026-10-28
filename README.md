# XCVario Pure

An ESP32-based, lightweight variometer system with an open data interface supporting OpenVario, Cambridge, Borgelt, Naviter, and XCVario protocols, designed for dedicated XCVario hardware.

![Vario](https://github.com/hjr/XCVarioPure/blob/master/XcvPure.png)

The project is rooted in the open-source and open-hardware flight computer community and is based on the ESP32 SoC, supporting modern digital sensors such as BMP280, SPL06-007, DS1820B, TE4525, ABPMRR, MP5004DP, AHRS MPU6050, and the QMC5883L magnetometer.

The variometer features a sunlight-readable 2.4-inch IPS LCD display with a brightness of 1000 nits and a low power consumption of less than 1.2 W, corresponding to approximately 100 mA at an operating voltage of 12.5 V. The device measures 64 (H) × 64 (W) × 35 (D) mm, fitting a standard 57 mm (2″) instrument gauge, and weighs approximately 170 grams.

## Vision
The widespread availability of high-fidelity sensors combined with today's highly integrated computing platforms provides an excellent foundation for a project like XCVario Pure. At the same time, it quickly becomes apparent that the overall quality of such a system is determined primarily by its software architecture. Developing a truly high-end variometer therefore requires a significant software investment. Continued improvements in quality, functionality, and usability cannot be achieved with a purely volunteer-driven "pick-it-for-free" approach. Sustained development requires both substantial engineering effort and a sustainable funding model.

The evolution from **XCVario** to **XCVario Pure** therefore comes with the addition of software expertise to the team. The open-source repository will retain all basic features, as it did initially. 
In addition, many features intended to create commercial value will remain closed-source and are therefore not part of this repository. The goal of evolving the variometer into a best-in-class product remains unchanged.

The original ambition of providing a high-quality variometer sensor that turns any club glider with an outdated variometer into a fun-to-fly glider remains unchanged. In addition, new capabilities are being introduced to address several long-standing limitations of conventional variometer designs. The objective is to reduce the pilot's cognitive workload by automating routine tasks, allowing more attention to be focused on safe and efficient cross-country flying.
New features are/will be:

* Auto calibration for TEK and sensor synchronisation/fusion.
  - resulting in uncompromising TE variometer accuracy with a single indicator
  - no requirement for a TE probe any more
  - overcoming the tube latency introduced hurdles from attached probes
* Extended acoustics 
  - morphing a traditional electronic variometer tone to a fully sonified variometer display.
  - assisting flap setting through sounds
  - reintroduces Peter Zander's thermal-assistance audio cues
  - add gust audio overlays to improve interpretation of variometer indications during high-speed cruise
  - route numerous warnings through the acoustic channel
* User interaction
  - refined for intuitive fingertip operation
  - convert numerical values into intuitive pilot guidance (for example, McCready: Slow / Medium / Best Glide)
* Instant wind
  - help the pilot remain aligned with thermal streets and cloud streets

## Downloads, images and video material

[Release builds](https://github.com/hjr/XCVarioPure/releases/)

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
* Optional landing gear warning sensor
* Optional GPS NMEA data stream
* Optional FLARM traffic NMEA data stream
* Optional external instant wind sensors (Anemoi)

The ESP32 integrates Bluetooth, Wi-Fi, and multiple serial interfaces, allowing the XCVario Pure to communicate with virtually any modern glide computer. Supported protocols include XCVario, OpenVario, Cambridge, Borgelt, and Naviter, enabling connected devices such as XCSoar, LK8000 and SeeYou to receive TE variometer data, barometric altitude, airspeed, and additional flight information.

## The manual

[Online Handbook in various languages](https://xcvario.com/docs-category/vario)

## Feature list
* Variometer display with adjustable range (1 m/s – 30 m/s) and damping (1 s – 10 s)
* QNH, ballast, bugs, and McCready adjustment, plus many other configurable options
* Vario sound generator with adjustable volume and deadband, plus setup options for tone style profiles
* Sound sequencer with a variety of sound bites for sonification of most visual variometer information
* Integrated loudspeaker with 2-watt audio power
* S2F (Speed-to-Fly) indicator with configurable MC, ballast, and bugs based on glider polars
* Includes aproximately 150 predefined glider polars (list: https://github.com/hjr/XCVarioPure/blob/master/components/glider/PolarTable.txt)
* IAS or TAS airspeed indication
* OAT (Outside Air Temperature) sensor
* Flap assistant with or without flap position sensor
* Battery voltage and status indicator
* Thermal assistant indicator for efficient thermal centering
* Wind rose indicator with optional second, instant wind pointer
* Air mass estimator: EKF fusion of IMU, AHRS, airspeed, baro altitude and GNSS velocity
  - instant 3D wind; the horizontal part drives the instant wind pointer (wind source "XCVario+Instant")
  - vertical air mass movement as vario mode "Air Mass" (or "Cruise-Air Mass", gross while circling), insensitive to horizontal gusts and independent of TE compensation; falls back to netto while not available
  - uses the static port directly, whatever the altimeter source setting; compensates the IMU lever arm and accounts for sideslip from the lateral acceleration
  - runs its own gyro propagated attitude, aided by air data instead of a plain accelerometer gravity reference: pull-ups and zooms do not tilt it, so a stick thermal shows less false climb than TE netto
  - damped with half the Vario Damping setting: it does not take gusts for lift, so it can show a thermal entry about a second earlier than TE netto
  - with Netto Mode "Super-Netto" the circling sink is added (needle, average and audio), showing the climb as if circling there
  - in Air Mass mode the speed to fly command uses the air mass movement as well, steadier in gusts and pull-ups than TE netto
  - works with the 1 Hz GNSS of the Flarm
  - learns airspeed scale, pitch/AoA offset and the GNSS latency in flight, saved every 5 minutes and after landing; learned values are discarded when the IMU calibration, airspeed calibration, glider or GNSS source changes
  - flapped gliders: the pitch/AoA offset is learned per flap position (flaps shift the fuselage angle of attack by degrees); with a flap sensor it follows the lever, without one the flap recommendation (the glider's flap speeds; the Flap-Assist display need not be enabled), which works as long as the flaps are set roughly as recommended
  - a calibrated compass aligns the heading before the first turn
  - optional `$PXCVA` NMEA output for evaluation, and "Wind to Navi" sends the averaged wind to the navigation computer (`$WIMWV`, Cambridge `!w`)
  - "Pull-up Comp." compensates the static port error like the TE eAdjustment: tune it in calm air until pull-ups show neither false climb nor sink
* Bluetooth, Wi-Fi, and two serial interfaces for external devices (XCSoar, LK8000, FLARM, Anemoi, other sensors …)
* Complete support for a secondary variometer (two-seater) with automatic routing to all connected devices
* High-precision barometric altimeter with 1 hPa (≈ 8 m) absolute accuracy and 0.1 m resolution
* Sunlight-readable, high-contrast 2.4-inch IPS display with 1000 nits brightness
* External switch for S2F / Vario mode
* Setup menu for customization of variometer features
* Support for the FLARM View display
* Output of NMEA data in OpenVario, Borgelt, Cambridge, Naviter, or XCVario format
* OTA (Over-the-Air) software updates via Wi-Fi and web browser
* Attitude and Heading Reference System (AHRS) available since the 2021 hardware revision
* Wind calculation in circling and straight flight (requires a connected GPS source)
* Electronic TE compensation, eliminating long TE tubing and TE probe limitations