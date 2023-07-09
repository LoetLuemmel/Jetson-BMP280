# Jetson-BMP280
```Environmental sensor BOSCH BMP280 on Nvidia Jetson devices```


I coul not find working code for the popular BMP280 environmental sensor for the Nvidia Jetson family.
The code for the I2C bus should be in C and not Python.

The only code I found was from Jim@jetsonhacks.com, but here just bitwise writing to LED segments or displays.

On the BOSCH Sensortec page I found a note from Mr. Gao (https://github.com/Gfast2/Bosch_BME680_Jetson) along with some code samples for a more recent environmental sensor, the BME680. I was happy, because this his code sample needs to set some sensor parameters on the I2C bus and also reads parameter values.

Still the Bosch sample code for my sensor just iddentifies the correct chip ID and always fails transmitting/receiving more than one Byte.

Would be great, if this code could be modified to a working sample.
Out there there is tons of code in Python and for Arduino or ESP32, but I want to estblish some samples in C for the Nvidia Jetson family.


Helping hands appreciated,

LoetLuemmel
