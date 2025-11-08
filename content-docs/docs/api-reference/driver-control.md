# Driver Control API

Low-level driver control for motor drivers and actuators.

## Overview

The Driver Control API provides direct access to motor drivers for advanced control scenarios.

## Driver Controller

```python
from robocon_sdk.driver_control import DriverController

driver = DriverController()
```

## Driver Configuration

### Set Parameters

```python
driver.configure(
    driver_id=0,
    parameters={
        'current_limit': 15.0,
        'voltage_limit': 24.0,
        'pwm_frequency': 20000
    }
)
```

## Status Monitoring

```python
status = driver.get_status(driver_id=0)
print(f"Temperature: {status.temperature}")
print(f"Current: {status.current}")
print(f"Voltage: {status.voltage}")
```

