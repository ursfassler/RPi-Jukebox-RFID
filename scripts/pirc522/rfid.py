import threading
import spidev
import RPi.GPIO as GPIO

class RFID(object):
    pin_rst = 22
    pin_ce = 0
    pin_irq = 18

    mode_idle = 0x00
    mode_transrec = 0x0C
    mode_reset = 0x0F

    act_anticl = 0x93

    reg_tx_control = 0x14
    length = 16

    antenna_gain = 0x04

    authed = False
    irq = threading.Event()

    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1000000

        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.pin_rst, GPIO.OUT)
        GPIO.output(self.pin_rst, 1)

        GPIO.setup(self.pin_irq, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.pin_irq, GPIO.FALLING,
                callback=self.irq_callback)
        if self.pin_ce != 0:
            GPIO.setup(self.pin_ce, GPIO.OUT)
            GPIO.output(self.pin_ce, 1)
        self.init()

    def init(self):
        self.reset()
        self.dev_write(0x2A, 0x8D)
        self.dev_write(0x2B, 0x3E)
        self.dev_write(0x2D, 30)
        self.dev_write(0x2C, 0)
        self.dev_write(0x15, 0x40)
        self.dev_write(0x11, 0x3D)
        self.dev_write(0x26, (self.antenna_gain<<4))
        self.set_antenna(True)

    def spi_transfer(self, data):
        if self.pin_ce != 0:
            GPIO.output(self.pin_ce, 0)
        r = self.spi.xfer2(data)
        if self.pin_ce != 0:
            GPIO.output(self.pin_ce, 1)
        return r

    def dev_write(self, address, value):
        self.spi_transfer([(address << 1) & 0x7E, value])

    def dev_read(self, address):
        return self.spi_transfer([((address << 1) & 0x7E) | 0x80, 0])[1]

    def set_bitmask(self, address, mask):
        current = self.dev_read(address)
        self.dev_write(address, current | mask)

    def clear_bitmask(self, address, mask):
        current = self.dev_read(address)
        self.dev_write(address, current & (~mask))

    def set_antenna(self, state):
        if state == True:
            current = self.dev_read(self.reg_tx_control)
            if ~(current & 0x03):
                self.set_bitmask(self.reg_tx_control, 0x03)
        else:
            self.clear_bitmask(self.reg_tx_control, 0x03)

    def card_write(self, data):
        back_data = []
        back_length = 0
        error = False
        irq = 0x00
        irq_wait = 0x00
        last_bits = None
        n = 0

        irq = 0x77
        irq_wait = 0x30

        self.dev_write(0x02, irq | 0x80)
        self.clear_bitmask(0x04, 0x80)
        self.set_bitmask(0x0A, 0x80)
        self.dev_write(0x01, self.mode_idle)

        for i in range(len(data)):
            self.dev_write(0x09, data[i])

        self.dev_write(0x01, self.mode_transrec)

        self.set_bitmask(0x0D, 0x80)

        i = 2000
        while True:
            n = self.dev_read(0x04)
            i -= 1
            if ~((i != 0) and ~(n & 0x01) and ~(n & irq_wait)):
                break

        self.clear_bitmask(0x0D, 0x80)

        if i != 0:
            if (self.dev_read(0x06) & 0x1B) == 0x00:
                error = False

                if n & irq & 0x01:
                    print("E1")
                    error = True

                n = self.dev_read(0x0A)
                last_bits = self.dev_read(0x0C) & 0x07
                if last_bits != 0:
                    back_length = (n - 1) * 8 + last_bits
                else:
                    back_length = n * 8

                if n == 0:
                    n = 1

                if n > self.length:
                    n = self.length

                for i in range(n):
                    back_data.append(self.dev_read(0x09))
            else:
                print("E2")
                error = True

        return (error, back_data, back_length)

    def request(self, req_mode=0x26):
        """
        Requests for tag.
        Returns (False, None) if no tag is present, otherwise returns (True, tag type)
        """
        error = True
        back_bits = 0

        self.dev_write(0x0D, 0x07)
        (error, back_data, back_bits) = self.card_write([req_mode, ])

        if error or (back_bits != 0x10):
            return (True, None)

        return (False, back_bits)

    def anticoll(self):
        """
        Anti-collision detection.
        Returns tuple of (error state, tag ID).
        """
        back_data = []
        serial_number = []

        serial_number_check = 0

        self.dev_write(0x0D, 0x00)
        serial_number.append(self.act_anticl)
        serial_number.append(0x20)

        (error, back_data, back_bits) = self.card_write(serial_number)
        if not error:
            if len(back_data) == 5:
                for i in range(4):
                    serial_number_check = serial_number_check ^ back_data[i]

                if serial_number_check != back_data[4]:
                    error = True
            else:
                error = True

        return (error, back_data)

    def irq_callback(self, pin):
        self.irq.set()

    def wait_for_tag(self):
        # enable IRQ on detect
        self.init()
        self.irq.clear()
        self.dev_write(0x04, 0x00)
        self.dev_write(0x02, 0xA0)
        # wait for it
        waiting = True
        while waiting:
            self.dev_write(0x09, 0x26)
            self.dev_write(0x01, 0x0C)
            self.dev_write(0x0D, 0x87)
            waiting = not self.irq.wait(0.1)
        self.irq.clear()
        self.init()

    def reset(self):
        authed = False
        self.dev_write(0x01, self.mode_reset)
