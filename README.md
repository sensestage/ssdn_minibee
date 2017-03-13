# Sense/Stage MiniBee Arduino files

**This is the version for Arduino 1.6.5 and upwards**

*for older versions check the branch [arduino_1-0](https://github.com/sensestage/ssdn_minibee/tree/arduino_1-0)*

This repository contains the hardware definitions and MiniBee firmware for the SenseStage boards to use within Arduino.
These adaptations are necessary for revision B (and up) of the SenseStage MiniBee, as this revision uses a different clock than any of the default Arduino boards.

For revision A (only in possession of a few early testers), you can select to use the “LilyPad Arduino w/ ATmega168″ or “Arduino Pro or Pro Mini (3.3V, 8 MHz), w/ ATmega168″ instead.

## To install

- Unarchive the package.
- In there you will find a folder called `hardware`.
- Copy the folder (including all the contents) to the root of your Arduino Sketch folder. If you have already a `hardware` folder in there, then copy the “minibee” folder that is in the downloaded package's `hardware` folder to it.
- You will also find a folder called `libraries` in the downloaded package.
- Copy the folder (including all the contents) to the root of your Arduino Sketch folder. If you have already a `libraries` folder in there, then copy the  `MiniBee` folder that is in the downloaded package’s `libraries` folder to it.
- (Re)start the Arduino IDE.
- The MiniBee Boards will show up in the Tools -> Board ->.
- The MiniBee Firmware examples will show up in the:
    AT-mode: File -> Examples -> MiniBee, and the MiniBee Firmware library will show up in Sketch -> Import Library… -> MiniBee
    API-mode (recommended) File -> Examples -> MiniBeeAPIn, and the MiniBee Firmware library will show up in Sketch -> Import Library… -> MiniBeeAPIn

**It is recommended to use the API version of the firmware, as this is the one that is currently being developed.**


See also:
[https://docs.sensestage.eu/minibee/prepare-the-arduino-ide-for-use-with-sense-stage.html]()
