# Python API

## Control Client
``` python

# Create a new kermit module
kermit = KermitControlModule()

kermit.start_kermit()

sleep(4)

kermit.set_linear(7.0)

sleep(5)

kermit.set_angular(5.6)
kermit.quit_kermit()

```
