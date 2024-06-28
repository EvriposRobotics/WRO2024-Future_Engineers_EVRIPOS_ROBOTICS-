**Electronic/electric components wiring diagram**
The color of the cable indicates the type of function/connection it performs:

**Caution!!!
For the electronic components of the vehicle to work properly, the ground of the nimh battery pack (4x AA) must be connected to one of the ground points (marked as GND) on the arduino board.**

For easier management and connection of the power supply lines of the electronic components we have created a prototype board on which we have soldered a line for the 5v power supplies (from the nimh batteries) and a line for the grounding. Also on the same board we have placed a switch which serves to start the robot. The one pin of the button is connected to the ground and the other to the digital port 12 of arduino, used as a input for this case. The same thing can be done using a breadboard.

We included a 5v relay board to the output of which we have connected the 5v power supply from the battery. The relay is normally open and when it is given a high signal command from digital output 2 of the arduino then the circuit is closed and the electronic components that require 5v voltage are powered. In this way we save significant amounts of energy from the nimh battery pack and our vehicle gets much more autonomy.
The only devices powered by 3.3v are the mpu6050 gyroscope and the apds-9960 color sensor.

 - Wires in gray color: the two USB cables that connect a)Arduino Uno with Raspberry Pi 3 (usb type B cable) and b)Raspberry Pi 3 with the powerbank (usb micro B cable)
 - Wires in purple color: the special caCSI type cable than connects Raspberry Pi 3 to the Raspberry Pi  camera module
 - Wires in red color: the power lines at 5v.
 - Wires in orange color: the power lines at 3.3v
 - Wires in black color: the ground lines
 - Wires in the other colors: various signal input-output cables connecting on Arduino Uno

The "datasheets" subfolder inludes the datasheets of the Sharp distance sensors. The datasheets are useful because the output voltage of these sensor do not have a linear response accordingly to the calculated distance.
