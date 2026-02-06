# ESP32_Anchor_Alarm
Anchor Alarm with LolinLite ESP32, GPS Module, ePaper Display and setting via rotary encoder

At anchor we like to use an anchor alarm app on the mobile. Unfortunately that one has a few disadvantages: it uses up the battery quickly, and sometimes Android likes to stop the app for mysterious reasons, particularly if it is not in the front and the screen on. We could also use nachor alarms embedded in other onboard electronics (e.g. AIS), but I find that to inflexible and using too much energy from our precious house batteries.

Thatˋs why I have come up with an alternative, an GPS based anchor alarm with LiPo battery and an ePaper display. It is run by power saving variant of the ESP32 SOC, the Lolin Lite.

![Housing1](https://github.com/88markus88/ESP32_Anchor_Alarm/blob/main/pictures/PXL_20260204_162428316.jpg | width=100)
![Housing2](https://github.com/88markus88/ESP32_Anchor_Alarm/blob/main/pictures/PXL_20260204_162439578.MP.jpg | width=150)
![Display](https://github.com/88markus88/ESP32_Anchor_Alarm/blob/main/pictures/PXL_20260204_164919321.jpg| width=200)

<img src="[https://github.com/favicon.ico](https://github.com/88markus88/ESP32_Anchor_Alarm/blob/main/pictures/PXL_20260204_164919321.jpg)" width="200">

Main components:
- Lolin Lite ESP32: optimized for battery operation while still being inexpensive. It is optimized for battery operation, the module has a very low consumption in deep sleep.
- ePaper Display 1.54" with 200x200 resolution. Sufficiently large for the use case, very low power consumption and pretty good readabilty
- 1800 mAh LiPo battery: sufficient for 30 hours of operation (measured power consumption is 60 mA on avarage)
- NEO-6M GPS module
- Optional: active GPS antenna
- Rotary Encoder with button for settings
- PCB board: I have recycled a board for the ePaper barograph that I built earlier. I had a few ones left over, all the required pins are exposed (some not properly labeled...)
- 3D printed housing 

