This library is originally based on https://github.com/vanadiumlabs/arbotix/tree/master/libraries/Bioloid.  
It retains the same underlying approach of emulating a half-duplex serial connection on an **Atmega644p** to control **Dynamixel 12A** servos, but is heavily refactored.
**NOTE** this branch is modified to work on UART3/Serial3 of an ATMega2560.

For information on the servos:
https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/

## 'software' half-duplex
This is the set up on master branch.

The RX and TX lines must be connected in hardware.  
The library controls whether the bus is in RX or TX state by changing the serial port settings accordingly (see Atmega datasheet). Because the lines are tied together, the microcontroller may receive data that it sent out - this is filtered out by the library. 
The response should always be read after a transmission else there is a risk of putting the bus into transmit mode when the servo is also trying to send a response. For this reason I suggest that the status return level is kept on the default (ALL).

## Hardware half-duplex option
See branch  
```sh
hardware_uart_to_half_duplex
```
It would be more convenient to combine this with master, but due to the use (at the arduino library level) of interrupts in the software half-duplex version, and the inability to provide compile definitions through the arduino IDE build, this works out to be pretty awkward (and hence not done).  
![half_duplex_wiring](docs/hardware_half_duplex.png)  
![half_duplex_wiring_example](docs/hardware_half_duplex_example.jpg)  
![hardware_half_duplex_lines](docs/hardware_uart_to_halfduplex.png)  


## Communication protocol
Several different instructions. Below is shown how the packets in the example/instruction sketch look from a logic analyser on the rx/tx wire.
Full manufacturer's info here: https://emanual.robotis.com/docs/en/dxl/protocol1/

### Ping
Ping and response  
![ping](docs/ping.png) 
![response](docs/ping_response.png)  
The response is received about 1ms after the request.
![ping_and_response](docs/ping_and_response.png)  

### Read register
Read register and response  
![read_reg](docs/read_reg.png) 
![read_reg_response](docs/read_reg_response.png)

### Write register
![write_reg](docs/write_reg.png)  

### Staged write
![response](docs/staged_write.png)  

### Trigger staged action
![response](docs/trigger_staged.png)  

### Sync write
![response](docs/sync_write.png)  

