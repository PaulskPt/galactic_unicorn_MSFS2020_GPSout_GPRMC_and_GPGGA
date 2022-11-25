# Pimoroni 'Galactic_Unicorn_MSFS2020_GPSout_GPRMC_and_GPGGA'
 Receive, fliter and dispay MSFS2020 GPS data on the 53 x 11 LED matrix display of the 'Galactic Unicorn'.


Display flown track to the display of the Galactic Unicorn and use ground speed to control the displayed data


Used hardware:

a) a Personal computer running Microsoft Windows 11 or a Microsoft XBox seriex X (not tested) on which are installed and running: 
    - (1) Microsoft Flight simulator 2020 (MSFS2020) (https://www.flightsimulator.com/);
    - (2) FSUIPC7 add-on app from Pete & John Dowson (http://www.fsuipc.com/);

b) `Pimoroni Galactic Unicorn (PIM 631)` <https://shop.pimoroni.com/products/galactic-unicorn?variant=40057440960595>

c) `Adafruit CP2021N Friend - USB to Serial Converter` (Product nr 5335, https://www.adafruit.com/product/5335) (signal levels are 3.3V);
   Other type of USB to RS232 serial converters I tested are:
   - Adafruit MCP2221A breakout - General Purpose USB to GPIO ADC I2C - Stemma QT / Qwiic (Product ID 4471 https://www.adafruit.com/product/4471).
   - Model YP-05. Attention: use a model that is able to provide or set for logic 3V3 level signals;
d) `STEMMA QT / Qwiic JST SH 4-pin to Premium Male Headers Cable - 150mm Long` cable (Product nr 4209, https://www.adafruit.com/product/4209)
e) a small breadboard to put on the CP2102N and connect the dupont male SH pins.

Flow of the GPS data:  PC MSFS2020 w FSUIPC7 > COMx > CP2102n TX/RX > to one of the two I2C connectors on the back of the Galactic Unicorn (preferebly #1).
```
Serial connection: CP2102N TX > Galactic Unicorn I2C SDA (yellow wire)
                   CP2102N RX > Galactic Unicorn I2C SCL (blue wire)
                   CP2102N 3V > Galactic Uniforn I2C Vcc (red wire)
                   CP2102N GND > Galactic Unicorn I2C GND (black wire)
```
This project uses micropython.

Goals of this project:

To receive, filter and use certain elements of GPRMC and GPGGA GPS datagrams data sent by an add-on called ```FSUIPC7``` to the ```Microsoft Flight Simulator 2020 (FS2020)```.
From the filtered GPRMC GPS datagram message this project uses the airplane's position in ```Latitude``` and ```Longitude```, the ```groundspeed``` and the ```Track made good true```. From the filtered GPGGA GPS datagram message only the ```Altitude``` data is used. 
In this moment only th flown track is shown on the LED matrix display.  When the groundspeed value is > 0.2 and <= 30 kts, the airplane is assumed to be taxying. If the groundspeed is 0, during a short period, the airplane is assumed to be stopped or parked. The states: 'ac parked' and 'taxying' are shown on the LED matrix display. As soon as the groundspeed exceeds 30 kts the flown track will be displayed onto the LED matrix display of the Galactic Unicorn.

This is a work-in-progress.
In later updates also other GPS data will become available to be displayed on the Galactic Unicorn:
```
- Latitude/Longitude;
- Altitude;
- Groundspeed
- Track (true) flown. 
```

NOTE: The baudrate is set to 4800 baud (inside the FSUIPC7 > GPSout > 1 (or > 2). There, also select the correct COM-port for MS Windows 11)

Data Indicator LED:
Many USB-to-Serial converters have a LED that signals the presence of data. The CP2102N and YP-5 listed under c) above have such a LED.

Programming of the Buttons of the Galactic Unicorn.
In this moment only the Zzz button on the right side (middle) is programmed to function as `reset` button.
Next versions of this script will use the A, B, C and D buttons to select which other GPS data will be displayed

This script uses the ntptime kernal module of micropython to connect to a ntp time server.
Please fill in the following in the file secrets.py:
- WiFi SSDI;
- WiFi Password;
- COUNTRY. e.g.: "PT" for Portugal. "USA" for United States of America. COUNTRY will be used in setting '.' for thousands and ',' for decimal marking if COUNTRY is "PT".
- TZ_OFFSET. Your timezone offset to UTC in hours;
- NTP_SERVER, when you want to use a local NTP-Server.

I used the Mu-editor app to save, edit and test the script file: ```code.py```. I also used VSCode to find bugs and to copy the list of variables and functions.


Disclamer:
This project has been tested and working on a pc running MS Windows 11 Pro.

Other sources:
To read more about the GPS GPRMC datagram specification see: ```https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm```.

License: MIT (see LICENSE file)
