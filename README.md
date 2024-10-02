# PicoGo Library for Circuitpython

This is a port of the [Micropython library](https://github.com/MKesenheimer/pico-go) for [Waveshare's PicoGo](https://www.waveshare.com/wiki/PicoGo) robot.

> [!NOTE]
> This library only works with the Raspberry Pi Pico's version of Circuitpython as the PicoGo can only be used with that board.

## Requirements

Place the following librarys in the `lib` folder on your Pico.
```
- adafruit_bus_device
- adafruit_hcrs04.mpy
- adafruit_st7789.mpy
- neopixel.mpy
```

## Examples

Drive forwards for 3 seconds then blink the first neopixel:
```python
from picogo import PicoGo
import time

go = PicoGo()

# Range forwards:  0 (stop)  1 (full speed)
# Range backwards: 0 (stop) -1 (full speed)
go.set_motors(0.5, 0.5)
time.sleep(3)
go.set_motors(0, 0)

for i in range(3):
    go.NEOPIXEL[0] = (255, 255, 255)
    time.sleep(0.5)
    go.NEOPIXEL[0] = (0, 0, 0)
    time.sleep(0.5)
```

Display the distance of the ultrasonic sensor on the display:
(Note that you'll need the `adafruit_display_text` library)
```python
import time, terminalio
from picogo import PicoGo
from adafruit_display_text import label

go = PicoGo()

text = label.Label(terminalio.FONT, text="", color=0xFFFFFF, y=10, scale=2)
go.DISPLAY.root_group = text

while True:
    text.text = f"Distance: {go.measure_distance()}"
    time.sleep(0.1)
```

The library uses standard Circuitpython objects. For documentation on these, look at https://learn.adafruit.com or https://docs.circuitpython.org.