# Firmware Documentation
![Static Badge](https://img.shields.io/badge/version-v1-orange)
![Static Badge](https://img.shields.io/badge/status-development-green)


### Introduction
Based on the problem statement, this device is a slave device because it implements slave functions especially the MODBUS register read and write. i.e:
- Read coils
- Write single coil
- write multiple coils

Also, I figured that because it has relays, these relay are to be controlled by another device (MASTER device) via the MODBUS RTU. And this master device is the SIEMENS S7-1200.

Also this slave exposes its port/URL via ethernet. So data can be transferred, logged etc..

## Functional Requirement
## 1.Relay Control

### Relay logical organization
To organize 32 relays logically, and to remain simple, I used bit groups. Each relay bank is a uint8_t type, where each relay is represented by a single bit in that uint8_t type.

|Bank name| Type| Relays range |
|---|---|---|
|BANK_0|uint8_t | Relay 0-7 |
|BANK_1|uint8_t | Relay 8-15 |
|BANK_2|uint8_t | Relay 16-23 |
|BANK_3|uint8_t | Relay 24-31 |

These are defined in ```relay.h``` file. Since there are 32 relays to be controlled, I use an expander via i2c for this function.

#### Expander driver
THe driver I use is the MCP23017. I choose it because it has 16 bit outputs whcih means I only need 2 expanders to handle 32 relays.
I wrote a basic driver for the needed fucntions of interfacing the RELAY to the STM32. Some of these fucntions are:

```c
void MCP_initialize(MCP23017_instance inst, I2C_HandleTypeDef* i2c_handle, uint8_t address);
void MCP_pinmode(MCP23017_instance inst, uint8_t pin, uint8_t mode);
void MCP_all_pinmode(MCP23017_instance inst, uint8_t state, uint8_t port);
void MCP_write_pin(MCP23017_instance inst, uint8_t pin, uint8_t level);
uint8_t MCP_read_pin(MCP23017_instance inst, uint8_t pin);
uint8_t MCP_read_port(MCP23017_instance inst, uint8_t port_num);
void MCP_clear_port(MCP23017_instance inst, uint8_t port_num);

```

#### Relay driver API
To meet hardware abstraction and code maintainability, all relay control functions are defined in their own header file,
```c relay.h```

I have written the API to achieve the required deliverables of setting, clearing and reading the state of each relay, using logical
orgnaization in banks.

The relay API functions are listed below:
```c

void relay_init();
void relay_set(uint8_t bank, uint8_t relay_num, uint8_t state);
boolean_t relay_read_state();
uint8_t relay_resolve_bank(uint8_t n);

uint8_t relay_read(uint8_t bank, uint8_t relay_num);
uint8_t relay_read_bank(uint8_t bank);

void relay_clear(uint8_t bank, uint8_t relay_num);
void relay_clear_bank(uint8_t bank);


```
The fucntions are pretty quite well documented on the files themselves.


#### Abstraction levels
To meet hardware abstraction, the MCP23017 is only visible through the relay driver APIs, since that is the only user. This diagram shows the abstraction:

![relay-abstraction](../images/relay-abstraction.png)

#### Scalability of the relay controls
Now, because I used an I2C expander, the MCP23017 has 3 address bits(A0,A1, A2), so it can handle up to 8 devices on the same I2C peripheral. Those are 128 relays that can be added.
Scaling the Relay control class is as "simple" adding another MCP23017 IC and hardware-configuring the address.
On the software side, the driver remains the same, but if we add more relays we need to add more banks. SHould be trivial. Rather, this was my approach.


## 2. MODBUS RTU
My implementation and handling of MODBUS RTU is as follow:

The following is the structure of a MODBUS RTU packet:



For efficiency due to handling a large data packet, I use UART with DMA for data reception.
The data is routed via MAX485 transceiver to handle TTL to RS485 conversion. Then my driver handles this next part which is written to parse/decode the packet:

#### MAX485-MODBUS driver
I wrote a basic driver to handle MODBUS data reception. This driver exposes the following API calls:



####


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



### Priority table and logic behind it
