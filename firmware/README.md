### Firmware Documentation

#### General RTOS tasks
These tasks are shared among the slave and master devices:
- x_device_get_diagnostics

##### x_device_get_diagnostics task
This task is used to collect general board/device data

The data that U collect is:
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



### MODBUS Master

### MODBUS Slave

### Priority table and logic behind it
