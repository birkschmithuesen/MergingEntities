Instructions about soldering / making a box for the ESP :

Components needed :
. 1 PCB 6x8cm
. 1 ESP32
. 1 Multiplexer TCA9548A

. 2 LEDs : one yellow and one red
. 12 4k4 resistors (4k or 5k is okay)
. 1 button
. 1 switch
. 6 female jack
. 6 male jack
. 6 MPU9250 or FXOS+FXAS
. 1 battery
. 1 strong glue to glue the box

Soldering the box and the pcb:
1. Solder the ESP and the Multiplexer to the PCB. See the positions on the box you already have
2. Link the 3v3 of ESP to the VIN of multiplexer, and the GND of ESP to the GND of multiplexer
3. Link SDA of multiplexer to D21 of ESP, link SCL of multiplexer to D22 of ESP
3. For each of the 6 female jack : link the SDA and SCL to one of the SCx and SDx of the multiplexer
4. Solder one resistors between each of the 12 SDA/SCL lines and the VIN
5. Link the 3v3 of ESP to each VIN of female jack, and the GND of ESP to each GND of female jack

6. Link the GND of the ESP to the V- of the battery
7. Link the VIN of ESP32 to on side of the switch, link the V+ of the battery with the other side of the switch
8. Link the V+ of red LED to D32 of ESP, link the V+ of the yellow LED to D33 of ESP
9. Link the GND of both LED to GND of ESP
10. Link one side of the button to the 3V3 of ESP, link the other side to D5 of ESP, link D5 of ESP to GND of ESP

Soldering the gyro to the jack :
. Be careful to link the V+ wire to the bottom of the female jack to connect V+ at the end


Notes :
. For the common components as battery, button, etc... you can use any kind of them, just be careful about the size to fit the holes
. On ESP32, VIN stands for VoltageInput, so this is how you power it when you are not on usb connections. To power others composents like LEDS, don't use VIN but 3V3 of ESP !
. Put the button in his hole BEFORE soldering it