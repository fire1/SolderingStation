# My DIY Soldering & Hot Air Station
This project is a smart controller for a Soldering Iron and a Hot Air Gun. It uses an Arduino to keep temperatures steady and safe.


#### Devices / Pin out:
    Device	    Component	        Pin
    Hot Air	    Heater (PWM)	    D9
                Fan (PWM)	        D11
                Temp Sensor	        A6
    S-r Iron	Heater (PWM)	    D10
                Temp Sensor	        A7
    Display	    LCD I2C	            A4 (SDA), A5 (SCL)


#### Libraries: 
- FastPID

- LiquidCrystal_I2C

- Dimmable_Light

Note:
* Fix Compile Error: If you see a "multiple definition" error, open the ArduinoSTL folder and comment out the std::nothrow line in new_handler.cpp.

#### Serial Commands
You can type these in the Serial Monitor to test:

`at` + Number: Set Hot Air temperature.

`it` + Number: Set Iron temperature.

`fp` + Number: Set Fan speed.