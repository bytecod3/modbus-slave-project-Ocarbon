# Firmware Documentation

### Introduction
Based on the problem statement, this device is a slave device becasue it implements slave functions especially the MODBUS register read and write. i.e:
- Read coils
- Write single coil
- write multiple coils

Also, I figured that because it has relays, these relay are to be controlled by another device (MASTER device) via the MODBUS RTU. And this master device is the SIEMENS S7-1200.

Also this slave exposes its port/URL via ethernet. So data can be transferred, logged etc..

## Functional Requirement
### Relay control
TO organize 32 relays logically, and to remain simple, I used bit groups. Each relay bank is a uint8_t type(a simple byte), where each relay is represented by a single bit in that uint8_t type.

|Bank name| Type| Relays range |
|---|---|---|
|BANK_0|uint8_t | Relay 0-7 |
|BANK_1|uint8_t | Relay 8-15 |
|BANK_2|uint8_t | Relay 16-23 |
|BANK_3|uint8_t | Relay 24-31 |

#### General RTOS tasks
These tasks are shared among the slave and master devices:
- x_device_get_diagnostics

##### x_device_get_diagnostics task
This task is used to collect general board/device data

The data that I collect is:
    - chip ID
    - Die temperature
    - core clock frequency
    - Internal supply voltages
    - Reference voltage (I read this incase ADC is used directly without external ADC converters that use I2C where internal ADC ref voltage does not matter much)
    - free heap size
    - minimum ever free heap size since started running
    - number of running tasks
    - TCB (task control block) size for each task
    - Reset cause (watchdog, software reset, brownout reset)

The inbuilt chip parameters can be enabled or disabled by setting the ```GET_INTERNAL_PARAMETERS``` to 0 in the ```custom_config.h``` file.



### MODBUS Slave

### Priority table and logic behind it
