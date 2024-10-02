import time, board, pwmio, digitalio as dio, analogio as aio, adafruit_hcsr04, neopixel, busio, fourwire, displayio, audiopwmio
from adafruit_st7789 import ST7789

def sleep_us(microseconds : int):
    start = time.monotonic_ns()
    while time.monotonic_ns() - start < microseconds * 1000:
        pass

class PicoGo:
    def __init__(self, np_auto_write : bool = True, use_onboard_ble : bool = True):
        # Motor
        self.MOTOR_PWM_A = pwmio.PWMOut(board.GP16, frequency=1000)
        self.MOTOR_PWM_B = pwmio.PWMOut(board.GP21, frequency=1000)
        self.MOTOR_A1 = dio.DigitalInOut(board.GP17)
        self.MOTOR_A1.direction = dio.Direction.OUTPUT
        self.MOTOR_A2 = dio.DigitalInOut(board.GP18)
        self.MOTOR_A2.direction = dio.Direction.OUTPUT
        self.MOTOR_B1 = dio.DigitalInOut(board.GP19)
        self.MOTOR_B1.direction = dio.Direction.OUTPUT
        self.MOTOR_B2 = dio.DigitalInOut(board.GP20)
        self.MOTOR_B2.direction = dio.Direction.OUTPUT
        
        # Front IR sensors
        self.IR_LEFT = dio.DigitalInOut(board.GP3)
        self.IR_LEFT.direction = dio.Direction.INPUT
        self.IR_RIGHT = dio.DigitalInOut(board.GP2)
        self.IR_RIGHT.direction = dio.Direction.INPUT
        
        # Ultrasonic sensor
        self.ULTRASONIC_SENSOR = adafruit_hcsr04.HCSR04(trigger_pin=board.GP14, echo_pin=board.GP15)
        
        # Neopixels
        self.NEOPIXEL = neopixel.NeoPixel(board.GP22, 4, auto_write=np_auto_write)
        for i in range(4):
            self.NEOPIXEL[i] = (0, 0, 0)
        
        # Battery sensor
        self.BATTERY = aio.AnalogIn(board.GP26)
        
        # ST7789 Display
        displayio.release_displays()
        spi = busio.SPI(clock=board.GP10, MOSI=board.GP11)
        while not spi.try_lock():
            pass
        spi.configure(baudrate=24000000) # 24MHz
        spi.unlock()
        self.DISPLAY_BUS = fourwire.FourWire(spi, command=board.GP8, chip_select=board.GP9, reset=board.GP12)
        self.DISPLAY = ST7789(self.DISPLAY_BUS, width=240, height=135, rotation=270, rowstart=40, colstart=53)
        
        # Buzzer
        # No working, because all timers are in use
        # self.SPEKER = audiopwmio.PWMAudioOut(board.GP4)
        
        # Bluetooth module
        # No Adafruit BLE, because CTS and RTS aren't connected
        self.BLE = busio.UART(board.GP0, board.GP1, baudrate=115200)
        
    def set_motors(self, a : float | None, b : float | None):
        if a != None:
            a = min(1, max(-1, a))
            if a == 0:
                self.MOTOR_PWM_A.duty_cycle = 0
                self.MOTOR_A1.value = False
                self.MOTOR_A2.value = False
            else:
                self.MOTOR_PWM_A.duty_cycle = int((-a if a < 0 else a) * 65535)
                self.MOTOR_A1.value = a > 0
                self.MOTOR_A2.value = a < 0
        if b != None:
            b = min(1, max(-1, b))
            if b == 0:
                self.MOTOR_PWM_B.duty_cycle = 0
                self.MOTOR_B1.value = False
                self.MOTOR_B2.value = False
            else:
                self.MOTOR_PWM_B.duty_cycle = int((-b if b < 0 else b) * 65535)
                self.MOTOR_B1.value = b < 0
                self.MOTOR_B2.value = b > 0
        
    def get_ir_status(self):
        return not self.IR_LEFT.value, not self.IR_RIGHT.value
    
    def measure_distance(self):
        try:
            return self.ULTRASONIC_SENSOR.distance
        except RuntimeError:
            return -1
    
    # Returns battery percentage 0-100
    def read_battery_percentage(self):
        # No freaking clue what this does, just copied it from the original
        return min(max((self.BATTERY.value * 3.3 / 65535 * 2 - 3) * 100 / 1.2, 0), 100)

if __name__ == "__main__":
    print("Initializing...")
    go = PicoGo()
    print("No Errors!")
