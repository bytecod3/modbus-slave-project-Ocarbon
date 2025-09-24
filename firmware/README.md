### Firmware Documentation

#### General RTOS tasks
These tasks are shared among the slave and master devices:
- x_device_get_diagnostics

##### x_device_get_diagnostics task
This task is used to collect general board/device data such as:
    - chip ID
    - Die temperature
    - core clock frequency
    - Internal supply voltages
    - Reference voltage (I read this incase ADC is used directly without external ADC converters that use I2C where internal ADC ref voltage does not matter much)
    - stack pointer levels
    - minimum high water mark
    - number of running tasks
    - TCB (task control block) size for each task
    - Reset cause (watchdog, software reset, brownout reset)

The inbuilt chip parameters can be enabled or disabled by setting the ```GET_INTERNAL_PARAMETERS``` to 0 in the ```custom_config.h``` file.



### MODBUS Master

### MODBUS Slave
