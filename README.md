# AltiDuo console (firmware)
AltiDuo that can be configured using the Bearconsole Android application
<img src="/pictures/altiDuo_kit.jpg" width="49%"> 
# Talking to the altimeter using command line
It is possible to use a terminal to send some sort of AT command to the altimeter. The commands allows you to get the altimeter config, change it . Those commands are also used by all the graphical interface.
You will need a USB cable or a bluetooth module

# Talking to the altimeter using graphical user interfaces
- an Android application that can run on any Android device that is running at least Android 4.2. The application is called [BearConsole](https://github.com/bdureau/BearConsole2)

# Communicating with the altimeter board
The AltiServo board can communicate using the serial connector. On the serial connector you can plug several devices:
- a USB/ttl module
- a bluetooth module
- a 3DR telemetry module

etc ....
Any serial module should work. The firmware will have to be changed depending on what you want todo. The standard firmware can work with the first 3 modules.

# Building the code
You will need to download the Arduino ide from the [Arduino web site](https://www.arduino.cc/). 
You have to load the Arduino Uno boot loader to your ATmega328 micro controller. 
Make sure that you download the [support library](https://github.com/bdureau/AltimetersLibs) for the BMP085 sensor and copy them to the Arduino library folder. To compile it you need to choose the Arduino Uno board and the correct USB port.
You will need to use a USB/TTL adapter to connect the altimeter to your computer, refer to the documentation.

# Hardware
You can either build the AltiDuo board as a kit or use an Arduino Uno/Nano/pro and make a shield. 
