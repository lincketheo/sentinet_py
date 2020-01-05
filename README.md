# Python Documentaition

# Modules:
***

## Sentinet
Modules that connect to sentinet_cpp
Primarily implimented under the hood, so there isn't much to get going

```python
from sentinet.kermit import KermitControlModule

# Starts the control module
# Everything happens under the hood,
# So no need for configuration
a = KermitControlModule()

# Give kermit a command velocity
a.safe_update(4.5, 6.7) # (linear, angular)
# Kermit is already publishing, so
# this update is all you need

# Retrieve kermit's state 
x, y, theta, v, w = a.safe_get_state()
# Note that you probably only want to
# get the state after a state update,
# You can also register a callback, see
# Theo for how to do this
```
## Simulation
A Place for simple simulations

## State Machine
## Localizer
Localizer is implimented in c++ (sentinet_cpp)
This localizer is not supported
