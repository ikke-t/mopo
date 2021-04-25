""" module to implement moped computer """

import array
import machine
import micropython
import utime
from writer_minimal import Writer
import gc


# emergency buffer for errors in interrupt handler
micropython.alloc_emergency_exception_buf(100)

class Rpm():
    """ Listen for given I/O pin for rising edge interrupts and count rounds per
         minute measuring the intervals."""

    ARRAYSIZE = 20

    def __init__(self, pin=12):

        # By default Spark signal pin is GPIO12 (D6)

        self.sparks = array.array('i', (0 for x in range(self.ARRAYSIZE)))

        self.no_sparks = 0
        self.prev_spark_ms = 0
        self.spark_write_index = -1
        self.spark_read_index = 0
        self.spark_count = 0
        self.spark_overflow = 0
        self.spark_disqualified = 0
        self.this_spark_ms = 0

        self.rpm_pin = machine.Pin(pin, machine.Pin.IN)
        self.rpm_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=self.cb_spark)

    def cb_spark(self, pin):
        """ This is interrupt handler for counting sparks from ignition signal.
            Each spark means engine makes one round. We use circular que to
            record time stamps in milliseconds for each spark. Spark signal
            is not clean, so we throw away all bounces which would be quicker
            than the engine is likely to spin."""

        self.this_spark_ms = utime.ticks_ms()

        # first ever spark?
        if self.spark_write_index == -1:
            self.spark_write_index = 0
            self.sparks[self.spark_write_index] = self.this_spark_ms
            self.spark_count += 1
        # filter away any bounces over 12k rpm (< 5ms)
        elif utime.ticks_diff(
                self.this_spark_ms,
                self.sparks[self.spark_write_index]
                ) > 4:
            if self.spark_count <= self.ARRAYSIZE:
                self.spark_write_index = (
                    self.spark_write_index + 1) % self.ARRAYSIZE
                self.sparks[self.spark_write_index] = self.this_spark_ms
                self.spark_count += 1
            else:
                self.spark_overflow += 1
        else:
            self.spark_disqualified += 1

    def avg(self):
        """Get average of RPM from times stored into circular buffer from interrupt"""
        spark_intervals_ms = 0
        rpm = 0
        if self.spark_count > 0:
            # disable interrupt to protect simultaneous write of spark_count
            irq_state = machine.disable_irq()
            rounds = self.spark_count
            # read average of spark intervals
            while self.spark_count:
                latest_spark_ms = self.sparks[self.spark_read_index]
                self.spark_count -= 1
                self.spark_read_index = (self.spark_read_index + 1) % self.ARRAYSIZE
                spark_intervals_ms += utime.ticks_diff(latest_spark_ms, self.prev_spark_ms)
                self.prev_spark_ms = latest_spark_ms
            # re-enable the interrupt
            machine.enable_irq(irq_state)
            avg_spark_interval_ms = spark_intervals_ms / rounds
            rpm = rounds * 1000 / avg_spark_interval_ms * 60
        return rpm

class Speed():
    """Use hall sensor interrupt to count tyre spinning. There might be
       several magnets in wheel, so the measure the distance in millimeters
       the tyre traverses between each tick. Or full distance, and divide it
       by count of magnets."""

    ARRAYSIZE = 10
    PIN_HALL = 13

    def __init__(self, hall_tick_dist_mm=None, hall_pin=13):
        # By default hall sensor pin is GPIO13 (D7)

        self.halls = array.array('i', (0 for x in range(self.ARRAYSIZE)))
        self.hall_write_index = -1
        self.hall_read_index = 0
        self.hall_count = 0
        self.hall_overflow = 0
        self.hall_disqualified = 0
        self.hall_total = 0
        self.this_hall_ms = 0
        self.prev_hall_ms = 0
        self.no_halls = 0
        self.hall = machine.Pin(hall_pin, machine.Pin.IN)
        self.hall.irq(trigger=machine.Pin.IRQ_FALLING, handler=self.cb_hall)
        # what is the tyre spin distance in millimeters between hall sensor tics
        # e.g. hall sensor ticks 6 times per wheel spinning once, wheel size 179 cm
        # 1790/6 = 298.xxx
        if hall_tick_dist_mm is None:
            raise Exception("You must enter tyre traverse distance per hall"
                            "sensor tick")
        self.dist = hall_tick_dist_mm


    def cb_hall(self, pin):
        """ This is interrupt handler for counting hall sensor from tyre.
            Each hall tick means the tyre rolls certain distance.
            We use circular que to record time stamps in milliseconds for each
            tick. Hall sensor signal might not be clean, so we throw away all
            bounces which would be quicker than the tyre is likely to spin."""

        self.this_hall_ms = utime.ticks_ms()

        self.hall_total += 1
        # first ever hall signal?
        if self.hall_write_index == -1:
            self.hall_write_index = 0
            self.halls[self.hall_write_index] = self.this_hall_ms
            self.hall_count += 1
        # filter away any hall sensor bounces over 100 km/h (< 36ms)
        elif utime.ticks_diff(self.this_hall_ms, self.halls[self.hall_write_index]) > 36:
            if self.hall_count <= self.ARRAYSIZE:
                self.hall_write_index = (self.hall_write_index + 1) % self.ARRAYSIZE
                self.halls[self.hall_write_index] = self.this_hall_ms
                self.hall_count += 1
            else:
                self.hall_overflow += 1
        else:
            self.hall_disqualified += 1
            self.hall_total -= 1

    def get_avg(self):
        """Calculate the average speed from hall sensor tick timestamps stored
           in circular buffer by interrupt."""

        hall_intervals_ms = 0

        if self.hall_count > 0:
            # disable interrupt to protect simultaneous write of hall_count
            irq_state = machine.disable_irq()
            hall_tics = self.hall_count
            # read average of hall intervals
            while self.hall_count:
                latest_hall_ms = self.halls[self.hall_read_index]
                self.hall_count -= 1
                self.hall_read_index = (self.hall_read_index + 1) % self.ARRAYSIZE
                hall_intervals_ms += utime.ticks_diff(latest_hall_ms, self.prev_hall_ms)
                self.prev_hall_ms = latest_hall_ms
            # re-enable the interrupt
            machine.enable_irq(irq_state)
            avg_hall_interval_ms = hall_intervals_ms / hall_tics
            # if more than 3 halls, count speed, if less, also utilise old speed value
            # km/h = tics * tick_distance_mm * 10 / used_time_ms / 36
            speed = hall_tics * self.dist * 10 / avg_hall_interval_ms / 36
        else:
            self.no_halls += 1
            speed = 0

        return speed

class Display():
    """Output information on SSd1306 oled display"""

    # define how SSD1306 display is connected
    PIN_SDA = 4
    PIN_SCL = 5
    PIXEL_HEIGHT = 64
    PIXEL_WIDTH = 128
    I2C_ADDRESS = 0x3c

    def __init__(self, scl_pin=None, sda_pin=None):
        # font created by:
        # ../micropython-font-to-py/font_to_py.py \
        #   /usr/share/fonts/google-droid/DroidSans.ttf \
        #   -x 20 droidsans20.py -c 0123456789,.kmhrpm/
        import droidsans20
        # font created by:
        # ../micropython-font-to-py/font_to_py.py \
        #   /usr/share/fonts/google-droid/DroidSans.ttf
        #   -x 48 droidsans48.py 0123456789,.
        # import droidsans48
        # import droidsans32
        import ssd1306

        if scl_pin is None:
            scl_pin = self.PIN_SCL
        if sda_pin is None:
            sda_pin = self.PIN_SDA
        self.i2c = machine.I2C(scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin))
        self.oled = ssd1306.SSD1306_I2C(
                        self.PIXEL_WIDTH, self.PIXEL_HEIGHT,
                        self.i2c, self.I2C_ADDRESS)
        self.font_small = Writer(self.oled, droidsans20)
        # self.font_big = Writer(self.oled, droidsans48)
        # self.font_big = Writer(self.oled, droidsans32)
        # self.font_big = self.font_small

    def rpm_speed(self, rpm, speed):
        """Draw rpm and speed screen"""
        self.oled.fill(0)

        self.font_small.set_textpos(10, 10)
        tmp = int(rpm/100)
        self.font_small.printstring(str(tmp/10) + " rpm")

        tmp = int(speed)
        self.font_small.set_textpos(40, 10)
        self.font_small.printstring(str(tmp) + " km/h")
        self.oled.show()

    #  def rpm(self, rpm):
    #      """Draw RPM screen"""
    #      self.oled.fill(0)
    #      font_width = 31

    #      self.oled.fill(0)
    #      if rpm < 10:
    #          pos = (124 - 1.2*font_width)//2
    #      else:
    #          pos = 124//2 - 1.2*font_width

    #      # self.font_big.set_textpos(3, int(pos))
    #      self.font_small.set_textpos(3, int(pos))

    #      # self.font_big.set_textpos(3, 20)

    #      # we use tmp to get one digit accuracy for rpms, e.g 5323 is 5.3
    #      tmp = int(rpm/100)
    #      # self.font_big.printstring(str(tmp/10))
    #      self.font_small.printstring(str(tmp/10))
    #      self.oled.show()

    def speed(self, speed):
        """Draw Speed screen"""
        font_width = 31

        self.oled.fill(0)
        if speed < 10:
            pos = (124 - font_width)//2
        else:
            pos = 124//2 - font_width

        # self.font_big.set_textpos(3, pos)
        self.font_small.set_textpos(3, pos)
        # self.font_big.printstring(str(speed))
        self.font_small.printstring(str(speed))
        self.oled.show()

    #  def dist(self, trip, total):
    #      """Draw ODO screen"""
    #      self.oled.fill(0)

    #      self.font_small.set_textpos(10, 30)
    #      self.font_small.printstring(str(trip) + " km")

    #      self.font_small.set_textpos(40, 30)
    #      self.font_small.printstring(str(total) + " km")
    #      self.oled.show()


class Button():
    """Implements a button which can be tracked for both one or sequential
       presses."""

    NORMAL_SLEEP = 300
    BUTTON_CHECK_SLEEP = 50
    BUTTON_BOUNCE_DELAY_MS = 50
    BUTTON_SERIES_PRESS_DELAY_MS = 200
    BUTTON_DOWN = 0
    BUTTON_UP = 1

    def __init__(self, pin=2):
        # By defualt button pin is GPIO2 (D4)
        self.button = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)
        self.main_loop_sleep = self.NORMAL_SLEEP
        self.button_press_time_ms = 0
        self.button_press_count = 0
        self.button_debounce_timer = machine.Timer(-1)
        self.button_waiting_bounce = False
        self.button_waiting_more_clicks = False
        self.button_series_click_timer = machine.Timer(-1)
        self.button_series_click_prev_value = 0
        self.button.irq(trigger=machine.Pin.IRQ_FALLING, handler=self.cb_button)


    def cb_button_series(self, timer):
        """ Check every BUTTON_SERIES_PRESS_DELAY_MS if button is
            pressed to count clicks. If twice here while button not pressed
            we stop counting"""

        if self.button.value() == self.BUTTON_DOWN:
            if self.button_series_click_prev_value == self.BUTTON_UP:
                self.button_press_count += 1
            self.button_series_click_prev_value = self.BUTTON_DOWN
        else:
            if self.button_series_click_prev_value == self.BUTTON_DOWN:
                self.button_series_click_prev_value = self.BUTTON_UP
            else:
                # too times here with no press, stop counting
                timer.deinit()
                self.button_waiting_more_clicks = False
                self.main_loop_sleep = self.NORMAL_SLEEP


    def cb_button_debounce(self, timer):
        """ we filter away bounces, and if button press detected, we move to
            timer mode to count sequential clicks."""

        if self.button.value() == self.BUTTON_DOWN:
            # we have valid button press
            self.button_waiting_bounce = False
            self.button_press_count = 1
            self.button_waiting_more_clicks = True
            self.button_series_click_prev_value = self.BUTTON_DOWN
            self.button_series_click_timer.init(
                period=self.BUTTON_SERIES_PRESS_DELAY_MS,
                mode=machine.Timer.PERIODIC,
                callback=self.cb_button_series)
        else:
            self.main_loop_sleep = self.NORMAL_SLEEP

        self.button_waiting_bounce = False


    def cb_button(self, pin):
        """ This interrupt is to start counting how many times button is pressed.
            We mark the button pressed here, and start process of counting for
            more clicks"""

        if not (self.button_waiting_bounce or self.button_waiting_more_clicks):
            # this is first notice button is pressed
            self.button_waiting_bounce = True
            self.button_press_time_ms = utime.ticks_ms()
            self.main_loop_sleep = self.BUTTON_CHECK_SLEEP
            self.button_debounce_timer.init(
                period=self.BUTTON_BOUNCE_DELAY_MS,
                mode=machine.Timer.ONE_SHOT,
                callback=self.cb_button_debounce)

    def pressed(self):
        """Tell how many times button has been pressed. If currently
           waiting for the series of presses, return -1."""
        irq_state = machine.disable_irq()
        if self.button_waiting_more_clicks:
            pressed = -1
        else:
            pressed = self.button_press_count
            self.button_press_count = 0
        machine.enable_irq(irq_state)

        return pressed

class Limiter():
    """We can reduce ignition by connecting the ignition drive signal to
       with given duty cycle."""

    # 1-1000 Hz
    FREQ = 250
    # Duty cycles for different modes
    LIMIT_DUTY = 128
    LIMP_DUTY = 768
    # Operation mode
    UNLIMITED = 0
    LIMITED = 1
    LIMP = 2

    LIMIT_RPM_HIGH = 6000
    LIMIT_RPM_LOW = 5800
    LIMP_RPM_HIGH = 7000
    LIMP_RPM_LOW = 6800

    LIMIT_SPEED_HIGH = 42
    LIMIT_SPEED_LOW = 40
    LIMP_SPEED_HIGH = 50
    LIMP_SPEED_LOW = 48

    def __init__(self, pin=15, mode=None):
        # Kill switch pin is GPIO15 (D8)
        self.pin = pin
        self.ign = machine.Pin(self.pin, machine.Pin.OUT)
        self.ign.off()
        self.mode = self.UNLIMITED
        if mode is not None:
            if mode == self.LIMITED:
                self.limit(self)
            elif mode == self.LIMP:
                self.limp(self)

    def limit(self):
        """Turn off some ignition to give indication we are near limits"""
        if self.mode == self.UNLIMITED:
            self.ign = machine.PWM(machine.Pin(self.pin))
            self.ign.freq(self.FREQ)
        self.ign.duty(self.LIMIT_DUTY)
        self.mode = self.LIMITED

    def limp(self):
        """Turn off most ignition to stay within limits"""
        if self.mode == self.UNLIMITED:
            self.ign = machine.PWM(machine.Pin(15))
            self.ign.freq(self.FREQ)
        self.ign.duty(self.LIMP_DUTY)
        self.mode = self.LIMP

    def free(self):
        """Allow full ignition"""
        if self.mode != self.UNLIMITED:
            self.ign.deinit()
            self.ign = machine.Pin(self.pin, machine.Pin.OUT)
            self.ign.off()
        self.mode = self.UNLIMITED

    def get_mode(self):
        """Return ignition mode"""
        return self.mode

def show():
    """We keep looping here forever"""
    gc.collect()

    print("mem1: ", gc.mem_alloc()) 
    rpm = Rpm(12)
    print("mem2: ", gc.mem_alloc()) 
    display = Display()
    print("mem3: ", gc.mem_alloc()) 
    speed = Speed(hall_tick_dist_mm=298)
    print("mem4: ", gc.mem_alloc()) 
    button = Button()
    print("mem5: ", gc.mem_alloc()) 
    limiter = Limiter()

    i = 0
    while True:
        i += 1
        print("mem ", i, ": ", gc.mem_alloc(), " free: ", gc.mem_free()) 
        mode = limiter.get_mode()

        button_presses = button.pressed()
        print("button presses: ", button_presses)
        if button_presses > 0:
            display.speed(button_presses)
            # display.rpm_speed(button_presses*1000, button_presses)
            print("button presses: ", button_presses)
            # flash the button presses
            utime.sleep_ms(500)

        rpm_avg = rpm.avg()

        kmh = speed.get_avg()

        if (mode != limiter.UNLIMITED and
                rpm_avg < limiter.LIMIT_RPM_LOW and
                kmh < limiter.LIMIT_SPEED_LOW):
                # we were restricted, but now are free to go
            limiter.free()

        elif (mode != limiter.LIMP and
              (rpm_avg > limiter.LIMP_RPM_HIGH or
               kmh > limiter.LIMP_SPEED_HIGH)):
                # we were not limping, but are now above limp
            limiter.limp()

        if (mode == limiter.UNLIMITED and
                (rpm_avg > limiter.LIMIT_RPM_HIGH or
                 kmh > limiter.LIMIT_SPEED_HIGH)):
                # we were unrestricted, but now are above limit
            limiter.limit()

        elif (mode == limiter.LIMP and
              rpm_avg < limiter.LIMP_RPM_LOW and
              kmh < limiter.LIMP_SPEED_LOW):
                # we were limping, but are now below LOW
            limiter.limit()


        # Normally we update display every time, but loop is tighter if we are
        # in middle of polling button clicks to get fast interaction.
        # In such case we don't update display every round.
        if button.main_loop_sleep == button.NORMAL_SLEEP:
            display.rpm_speed(rpm_avg, kmh)
            print("rpm: ", rpm_avg, " kmh: ", kmh)
        elif i % (button.NORMAL_SLEEP // button.BUTTON_CHECK_SLEEP) == 0:
            print("rpm: ", rpm_avg, " kmh: ", kmh)
            display.rpm_speed(rpm_avg, kmh)

        utime.sleep_ms(button.main_loop_sleep)

if __name__ == "__main__":
    show()
