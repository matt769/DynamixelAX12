This library is based on https://github.com/vanadiumlabs/arbotix/tree/master/libraries/Bioloid.  
It retains the same underlying approach of emulating a half-duplex serial connection on an **Atmega644p** to control **Dynamixel 12A** servos, but is heavily refactored.

Note:  
The RX and TX lines must be connected in hardware.  
The library controls whether the bus is in RX or TX state by changing the serial port settings accordingly (see Atmega644p datasheet). Because the lines are tied together, the microcontroller may receive data that it sent out - this is filtered out by the library. 
The response should always be read after a transmission else there is a risk of putting the bus into transmit mode when the servo is also trying to send a response. For this reason I suggest that the status return level is kept on the default (ALL).



https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/