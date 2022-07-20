# Maynuo M9[78]xx Library

This repo contains a simple python3-based implementation of a wrapper for controlling
Maynuo electronic loads. My load is an M9812. It is the only one I have tested.

# Setup

`maynuo.py` is a simple library with few extra dependencies, except for
`serial` which you can install via pip or your package manager, and `crcmodbus`,
which you can install with pip.

# A little warning

Most of the Maynuo products have a DB-9 connector on the back that looks like
it is for RS-232. Though it is for serial, the voltage levels it expects are
TTL, _not RS-323_, which uses 12V signals. If you hook up a "real" serial port to
this connector, you will be unhappy.

You can use any of the common FTDI or similar USB-to-serial adapters, and wire
them into a DB-9 cable. Alternatively, you can purchase one of Maynuo's cables
for the purpose, which also provide galvanic isolation.

# Use

Using the library is hopefully straightforward.

First, instantuate a library object:

```python3
import maynuo

port        = '/dev/ttyUSB0'
speed       = 9600
device_addr = 1

m = maynuo.MaynuoLoad(port, speed, device_addr)
```

Note that the `speed` and `device_addr` can be adjusted from the front panel
of your load.

Once you have an object to work with, you can just issue it commands. For
example, do run down a battery, you can do:

```python3

v_terminal = 3.000
i_run      = 1.000

m.setReg('BATT', 0)
m.battTest(i_run, v_terminal)
m.inputOn()
while True:
    op = m.getOperatingPoint()
    print(op)
    if op['v'] <= v_terminal or op['i'] <= 0:
        break

```

Note that in battery mode, you have to turn the input on,
but the device itself will turn the input back off when
the voltage drops below the terminal voltage you specified.

In addition to battery discharge mode, you can easily set
up any of the other simple modes with `.setCC(current)`,
`.setCV(voltage)`, `.setCR(resistance)`, and `.setCP(power)`.

You can also access any named register on the device using
`.getReg(n)` and `.setReg(n,v)`

There are many other modes and functions on these units that
I have not explored, but it should be possible to access them
all by the register interface provided.


# Acknowledgements

This code was based in part on [MaynuoPi](https://github.com/harvie256/MaynuoPy)
