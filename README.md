# ESP32_Anchor_Alarm
Anchor Alarm with LolinLite ESP32, GPS Module, ePaper Display and setting via rotary encoder

At anchor we like to use an anchor alarm app on the mobile phone. Unfortunately that one has a few disadvantages: it uses up the battery quickly, and sometimes Android likes to stop the app for mysterious reasons, particularly if it is not in the front and the screen on. We could also use nachor alarms embedded in other onboard electronics (e.g. AIS), but I find that to inflexible and using too much energy from our precious house batteries.

Thatˋs why I have come up with an alternative, an GPS based anchor alarm with LiPo battery and an ePaper display. It is run by power saving variant of the ESP32 SOC, the Lolin Lite.

<img src="./pictures/PXL_20260204_162428316.jpg" width="300">
<img src="./pictures/PXL_20260204_162439578.MP.jpg" width="300">
<img src="./pictures/PXL_20260204_164919321.jpg" width="300">

Main components:
- Lolin Lite ESP32: optimized for battery operation while still being inexpensive. It is optimized for battery operation, the module has a very low consumption in deep sleep.
- ePaper Display 1.54" with 200x200 resolution. Sufficiently large for the use case, very low power consumption and pretty good readabilty
- 1800 mAh LiPo battery: sufficient for 30 hours of operation (measured power consumption is 60 mA on avarage)
- GPS module (Ublox NEO-6M, NEO-M8N or alternatively ATGM336H)
- Optional: active GPS antenna
- Rotary Encoder with button for settings
- PCB board: I have recycled a board for the ePaper barograph that I built earlier. I had a few ones left over, all the required pins are exposed (some not properly labeled...)
- 3D printed housing

## Using the Anchor Alarm
Switch on with the toggle switch on the side.
The device will go through a few selftests (including some beeps) 
Then the main menu is shown. Rotate the knob to move through the menu or change values, press it to select.

1: Program name and version<br>
2: Present GPS position in decimal notation<br>
3: Battery voltage and percentage<br>
5: Anchor Bearing - True compass bearing from boat to anchor<br>
6: Anchor Distance - Distance from boat to anchor om meters<br>
7: Alert Threshold - Numeric value that determines how often and how far the boat must be out of the safe circle before an alert is triggered. 10% out of the circle adds 1, 30% out of the circle adds 3 and so on. If the sum of these values is bigger than the Alert Threshold, the alert is triggered<br>
8: Alert Distance - Radius of the safe circle around the anchor in meters <br>
9: Sleeptime - Time in seconds that the device sleeps between measurements (unless there is an alert, then it does not go to sleep)<br>
10: Detail Info - Determines how much data is shown in the watch (routine) screen<br>
11: Graph Weight - Determines how thick the lines and symbols are drawn in the watch screen<br>
12: Power Saving Mode - Determines how aggressive power saving is applied
12: Exit: Exit the menu, change to measurement screen and starts the watch<br>

The watch screen shows the anchor at the center (cross), the safe circle around it and the boat as a "target" in relation to the anchor. Depending on the detail info selected, further information is shown:
- Anchor positon (top) and boat position (bottom). Previous position are shown as dots, size depending on the setting "Graph Weight".
- Number of satellites and HDOP of GPS
- Accumulated alert value (if bigger then Alert Threshold: Alert is triggered)
- Measurement Counter, how often did the device wake up and checked the position
- Battery voltage and percentage
- Distance from and bearing from initial anchor position to boat. (If the boat is SW of the anchor position, the bearing is 235°)

## Power Saving Modes
Since the device has limited battery capacity, power saving is available. Thre modes can be switched in the setup menu. Depending on the GPS module which is installed, different measures are taken:
| Mode          | NEO-6M                      | NEO-M8N                      |
| ------------- | -------------               | -------------                |
| MIN           | 80 MHz, NMEA Msg, Ped       | 80 MHz, NMEA Msg, Ped        |
| MID           | 80 MHz, NMEA Msg, Ped, PSM  | 80 MHz, NMEA Msg, Ped, PM2   |
| MAX           | 80 MHz, NMEA Msg, Ped, PM2  | 80 MHz, NMEA Msg, Ped, HF Off|

* 80 MHz: Reduce clock frequency for ESP32 from 240 to 80 MHz
* MEA Msg: Switch off all NMEA messages that are not neede. ONly RMC, GGA, GLL are left on (CFG-MSH
* Ped: Switch the GPS module to pedestrian mode, for better precision at slow speeds (CFG-NAV5)
* PSM: Switch GPS Module to PowerSavineMode (CFG-RXM)
* PM2: Configure extended Power Management to longer updatePeriod (CFG-PM2)
* HF Off: Switch off the GPS module HF section (CFG-RST). Biggest savings, but required time to re-acquire satellites

The effect of power saving (80 MHz) reduces the ESP32 power consumption to ca. 75 mA while not in deep sleep.<br>
The power consumption of the NEO-M8N is reduced to 45 mA in "MIN" mode to 28 mA in "MAX" mode.<br>
Precision of position is better in MIN mode than in MAX mode - depending on the quality of satellite data.

### Sleep Modes and Power Consumption
Power consumption has been measured using a FNIRSI FN-058 USB Power Meter, and theoretical runtime calculated based on a 1800 mA LiPo battery.

NEO-M8N 
| Mode  | Power (20s)|Endurance| Power (50s)|Endurance|
| ------| --------   |---------|---------   |---------|       
| MIN   | 53.0 mA    | 33.9 h  | 49.2 mA    | 36.6 h  |
| MID   | 39.5 mA    | 45.6 h  | 38.8 mA    | 47.4 h  | 
| MAX   | 43.5 mA    | 41.4 h  | 36.3 mA    | 49.6 h  |

Power consumption for the NEO-6M module is slightly higher, for the ATGM336H still higher, with correspondingly shorter operation times. All modules exceed 24 hours of continuous operation. With a USB power supply connected it is of course infinite.

### Additional Considerations
NEO-M8N appears the best module of all three used: lowest power consumption which can be further reduced using the extensive power management options. It uses satellites from GPS, Beidou, Glonass and Galileo constellations - which means shorter acquisiton times and better fixes. It is also the most expensive module.
Note: in MID mode this module only uses GPS satellites, with somewhat reduced position accuracy. In MIN and MAX mode it uses all three constellations of GPS satellites.

NEO-6M uses only GPS satellites, and has a slightly higher power consumption than NEO-M8N. It also has good power saving options. It is significantly cheapter than the NEO-M8N, and absolutely sufficient if the receiving conditions are good (no mountains or houses around which cover a large part of the sky).

ATGM336H uses GPS, Beidou and Glonass constellations. However, the position walk in my tests was significantly larger than for the NEO-M8N. Power saving options are fairly limited: NMEA messages can be reduced, and "pedestrian" mode can be used. The power saving option had no effect. Consequently this module has the highest power consumption of all three GPS modules. However, the GPS accuracy is good if the conditions are good and an active antenna is used instead of the mini-antenna supplied with the module. When the original antenna is used, acquisiton times can be many minutes if the reception is not good. Prices are low, comparable to the NEO-6M.

### GPS Performance
The following pictures show performance of the GPS with NEO-M8N and ARGM336H modules. The data have been collected in a park, with some trees above. Acquistion was a few seconds in every case. Power save mode "MIN" has been used in the first row, "MID" in the second row. After collecting for a few minutes I have moved about 20 meters to get the ship symbol away from the data point.
|Ublox NEO-M8N| ATGM336H |
|:---|:---|
|<img src="./pictures/2026-02-24 NEO-M8N MIN.jpg" width="300">|<img src="./pictures/2026-02-24 ATGM336 MIN.jpg" width="300">|
|<img src="./pictures/2026-02-24 NEO-M8N MID.jpg" width="300">|<img src="./pictures/2026-02-24 ATGM336 MID.jpg" width="300">|
The NEO-M8N limits itself to 12 satellites, ATGM336H uses as many satellites as it can get. In both cases the results are excellent for "MIN", with a somewhat larger distribution for the ATGM336H. The distribution is a bit wider in the "MID" mode, but still pretty good. The numbers in the third row from the bottom show the standard deviationof the data points in X and Y direction.

## Alert condition 
An alert is triggered if any of the following conditions is true:
- The present boat positon if too far from the original anchor position. Mathematical: (alert value) > (alert threshold). The alert value is accumulated over multiple runs: 10% out of the circle adds 1, 30% out of the circle adds 3 and so on. If the alert threshold is 10, an alert is triggered if the boat is out of the safe circle by more than 100% of the circle radius, or three times in sequence more than 33.3%, or ten times in sequence more than 10%. Whenever the boat returns into the safe circle, alert value is reset to 0.
- No valid GPS position for more than 10 seconds. This is reset as soon as a valid GPS position has been found again
- Battery voltage drops below 3.6V or battery percentage drops below 10%. Even with low voltage the device continues to work, until the battery protection kicks in

An alert can be silenced by pressing the button of the rotary encoder, the device then goes to the menu screen. 

## Schematic
Presently the schematic is available as Fritzing file. 
<img src="./Fritzing/ePaperAnkeralarm-Fritzing.jpg">

## Electrical Connections:
| KY-040| .     |
| ----- |------ |
| VCC | 3.3V |
| GND | GND |
| SW  | GPIO26 |
| DT  | GPIO 27 |
| CLK | GPIO 22 |

|Voltage measurement | .   |
| ----- | ------ |
| GPIO | 39 |

| Epaper | .   |
| ----- | ------ |
| BUSY | GPIO 04 [15] |
| RST  | GPIO 16 [RES, 2] |
| DC   | GPIO 17 [D/C] |
| CS   | GPIO 05 [SS, 4] |
| CLK  | GPIO 18 [SCK, SCL, 18] |
| DIN  | GPIO 23 [MOSI, SDA, Data/ 23] |
| GND  | GND |
| VCC  | 3.3V |

| Buzzer  | .   |
| -----   | ------ |
| Base    | via 2 K Ohm -> GPIO2 |

| NEO-6M / M8N GPS | .   |
| -----            | ------ |
| RX               |  GPIO15 (SDA) |
| TX               | GPIO19 (SCL) |
| GND              | GND |
| 3.3              | 3.3 V |

## PCB - Printed Circuit Board
For my first trials I have re-used a board that I previously used for another project. It does not fit exactly, but all required GPIOs are exposed.
ePaper: is connected as intended
Buzzer: is connected as intended via a S8050 transistor. The base is connected via a 2K resistor to GPIO2
NEO-6M GPS module: the connection initially intended for I2C is used (3.3V, GND, GPIO 15 (labeled SDA) and GPIO19 (labeled SCL)
KY-040 rotary encoder: is connected to 3.3V and GND via a OneWire connector, and to GPIOs 22,26 and 27 via the unmarked spare connectors on the left side of the Lolin Lite board

## Housing
Housing is 3D printed using PLA filament. The STL files for the print are included in the directory "STL"

## Disclaimer
This project is a personal hobbyist project and is provided for educational and experimental purposes only.

All hardware designs, software, firmware, and documentation are provided “as is” and “as available”, without any warranties, express or implied, including but not limited to warranties of merchantability, fitness for a particular purpose, or non-infringement.

The authors and contributors shall not be held liable for any direct or indirect damages, including but not limited to hardware damage, data loss, personal injury, fire, electrical hazards, or legal issues arising from the use, misuse, or inability to use this project.

This project is not intended for use in safety-critical, medical, automotive, industrial, or commercial applications.

You are solely responsible for verifying the correctness, safety, and suitability of this project for your use case, as well as ensuring compliance with all applicable laws, regulations, and standards.

Use this project at your own risk.
