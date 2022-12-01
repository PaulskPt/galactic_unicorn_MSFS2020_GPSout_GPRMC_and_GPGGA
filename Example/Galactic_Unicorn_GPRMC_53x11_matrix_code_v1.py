#!/usr/bin/python3
"""
    FOR USE WITH MICROPYTHON 

    Project to display Microsoft Flightsimulator airplane track (and evt other gps data) on a Pimoroni Ltd. Galactic Unicorn.
    This project was originally created to display Microsoft Flightsimulator airplane track on an Adafruit 64x32 RGB LED Matrix Panel
    controlled by a Pimoroni Interstate 75 controller using MicroPython programming language.
    Date of start of this project: 2022-01-14 20h10 PT.
    Date of start of the port of this project to the Galactic Unicorn: 2022-11-11.
    Hardware used:
    - Galactic Unicorn, with on-board Raspberry Pi Pico W (Micro-USB powered) controller (Product ID with Pimoroni Ltd: PIM631) £ 49.50;
    - Adafruit CP2104N Friend USB-to-Serial converter (Product ID: 5335):
      Alternative for the CP2104N: Adafruit MCP2221A, Gen Purpose USB to GPIO ADC I2C Stemma QT/Qwiic breakout (Product ID: 4471);

    Examples of GPS datagram messages of type GPRMC and GPGGA:

        b'$GPRMC,151948.00,A,5031.8614,N,00005.2524,E,83.0,315.1,201122,0.5,E*6A\r\n'
        b'$GPGGA,151948.00,5031.8614,N,00005.2524,E,1,05,0.0,914.4,M,0.0,M,0.0,0000*77\r\n'
        
    Function ck_uart() tries to read a full line through uart.readline()
    The data received is a bytearray. This bytearray is converted into a text buffer (rx_buffer_s).
    In the case the received data results in a None for 100 times, the function nodata() will be called
    which displays "nodata". If more than 1000 times there is no data, the function ck_uart() will exit
    with a value of 0.
    
    The received GPRMC GPS datagram will be split into twelve data items, saved as a GPRMC_lst list.
    The received GPGGA GPS datagram will be split into fifteen data items, saved as a GPGGA_lst list.
    The groundspeed data is used to discern if the airplane is moving or not. 
    When the groundspeed is zero, the text 'ac parked' is displayed. When the groundspeed is between
    0.2 and 30 KTS the text 'taxying' is displayed. Above a groundspeed of 30 KTS the flown track,
    the airplane's position, the groundspeed or the altitude will be displayed, depending the choice
    the user made, using the A or B button.

    Update 2022-07-23.
    Updated the micropython firmware to v1.19
    Modified the appearance of DIGIT nrs: '6' and '9'.
    Some mods in ck_uart and added gc module.

    NOTE: if you get errors or garbled / delayed reception of gps messages: Check the baudrate in FSUIPC GPSout
    and here in this script both are set for 4800 baud.
    
    Clock synchronizes time on start.
        
    Update 2022-11-30:
    IMPORTANT NOTE:
    Only connect the SDA, SCL and GND wires of the Galactic Unicorn I2C connector (#1 or #2)
    to the USB-to-serial connector device, e.g. a CP2102N.
    Do not connect the I2C red wire from the Galactic Unicorn to the +3V pin of the USB-to-serial converter device. 
    Connecting the red wire will prevent the Galactic Unicorn to reset properly:
    - MS Windows e.g. will report an error that the connected USB device has not been recognized;
    - The Thonny IDE 'stop/restart' button will not work and you will not be able to reach files on the board's filesystem. or #2) have to
    be connected to the USB-to-serial connector device, e.g. a CP2102N.

"""
import gc
#import time
import math
# ------------------------------------+
# Imports for Pimoroni DisplayHatMini |
# ------------------------------------+
import sys
import time
from array import *
from time import sleep, ticks_ms

from micropython import const
import machine
import network
import ntptime
# Imports for the Galactic Unicorn:
from galactic import GalacticUnicorn
from machine import RTC, UART, Pin, reset
from picographics import PicoGraphics, DISPLAY_GALACTIC_UNICORN 
from pimoroni_i2c import PimoroniI2C  # builtin in Pimoroni's micropython

from GU_Workout_mod_ini import *

try:
    from secrets import WIFI_PASSWORD, WIFI_SSID, TZ_OFFSET, NTP_SERVER
    wifi_available = True
except ImportError:
    print("Create secrets.py with your WiFi credentials to get time from NTP")
    wifi_available = False

# -------------------------------+
# General debug flag             |
my_debug = False  # Debug flag   |
# -------------------------------+
# Other important flags          |
# -------------------------------+
use_sound = False

# create galactic object and graphics surface for drawing
gu = GalacticUnicorn()
gr = PicoGraphics(display=DISPLAY_GALACTIC_UNICORN)

button_a_pressed = False 
button_b_pressed = False
button_c_pressed = False
button_d_pressed = False

button_a = machine.Pin(gu.SWITCH_A, machine.Pin.IN, machine.Pin.PULL_UP)
button_b = machine.Pin(gu.SWITCH_B, machine.Pin.IN, machine.Pin.PULL_UP)
button_c = machine.Pin(gu.SWITCH_C, machine.Pin.IN, machine.Pin.PULL_UP)
button_d = machine.Pin(gu.SWITCH_D, machine.Pin.IN, machine.Pin.PULL_UP)
if use_sound:
    button_vol_up = machine.Pin(gu.SWITCH_VOLUME_UP, machine.Pin.IN, machine.Pin.PULL_UP)
    button_vol_dn = machine.Pin(gu.SWITCH_VOLUME_DOWN, machine.Pin.IN, machine.Pin.PULL_UP)
button_lux_up = machine.Pin(gu.SWITCH_BRIGHTNESS_UP, machine.Pin.IN, machine.Pin.PULL_UP)
button_lux_dn = machine.Pin(gu.SWITCH_BRIGHTNESS_DOWN, machine.Pin.IN, machine.Pin.PULL_UP)
button_rst = machine.Pin(gu.SWITCH_SLEEP, machine.Pin.IN, machine.Pin.PULL_UP)

func_dict = {
    0: "pos_func",
    1: "gs_func",
    2: "crs_func", 
    3: "alt_func"
}

func_rev_dict = {
    "pos_func": 0,
    "gs_func": 1,
    "crs_func": 2,
    "alt_func": 3
}

curr_func = 2  # default function = disp_crs()
old_func = curr_func

brill = 100 # Using brill to make default brilliance less strong

gc.enable() # Enable autmatic garbage collection


width = gu.WIDTH
height = gu.HEIGHT

lcd_maxrows = height-1

rtc = None

ribbon = None

# +----------------------+
# | Definition for I2C   |
# +----------------------+
try:
    PINS_GC = {"sda": 4, "scl": 5}  # i2c Pins for Galactic Unicorn (RPi Pico W)
    i2c = PimoroniI2C(**PINS_GC)
except Exception as e:
    print("Error while creating i2c object", e)

# +----------------------+
# | Definition for UART  |
# +----------------------+
# 
# via a TTL-to-USB converter connected to desktop PC Paul1 (COM9)
#
try:
    uart = UART(1, 4800)

    # wait a minimum amount of time before trying to read the device
    sleep(0.25)
except ValueError:  # e.g.: "bad TX pin"
    uart = None
    pass  # for the sake of debugging the rest of this script, let go!

# Global definitions
# +--------------------------------------------+
max_lp_cnt = 14  # <<<=========== LOOP COUNT   |
# +--------------------------------------------+
if sys.version_info > (3,): # Checked 2101-06-02: sys.version_info results in: (3, 4, 0)
    long = int  # See the notation: '100L' below at line 503
# Note: sys.stdin  results in '<io.FileIO 0>'
#       sys.stdio  results in '<io.FileIO 1>'
#       sys.stderr results in '<io.FileIO 2>'

ID_s = "" # The datagram ID e.g. GPRMC

led = machine.Pin(25, machine.Pin.OUT)  # The led of the RPi Pico is just a mono light

led_interval = 1000
biLdIsOn = False
lp_cnt = 0
lstop = 0
startup = -1
previousMillis = 0  # unsigned long
loop_time = 0
# Message serial nr
msg_nr = 0
nRMC = None
nGGA = None
GPRMC_cnt = 0
GPRMC_lst = []
GPGGA_lst = []
le_GPRMC_lst = 0
le_GPGGA_lst = 0
nr_msg_items = 0

ac_no_data = 0
ac_stopped = 1
ac_taxying = 2
ac_flying = 3
ac_stat = ac_stopped

# next four defs copied from:
# I:\pico\paul_projects\pico\circuitpython\msfs2020_gps_rx_picolipo\2021-09-03_16h49_ver
lMagnetic = True
lTrackDirChgd = False
v_gs = 0

gs_old = 0.0
max_text_len = 25
# Buffers
rx_buffer_len = 256 # was: 160 en daarvoor: 2**5  = 2<<5 = 64. Also used: 120
rx_buffer = bytearray(rx_buffer_len * b'\x00')
rx_buffer_s = ''
msg_lst = [] # Create an empty message list. Will later be filled with the GPRMC message parts splitted
s_telapsed = "Time elapsed between uart rx and GU matrix presentation: "

# Pre-definition to prevent ... isn't defined
#def scroll_text(msg):
#    pass

#----------------------------- for Galactic Unicorn -----------------
# Copied from script: scrolling-text.py 
#
# constants for controlling scrolling text
PADDING = 5
MESSAGE_COLOUR = (10, 0, 96) # was (255, 255, 255)
OUTLINE_COLOUR = (0, 0, 0)
BACKGROUND_COLOUR = (0,0,0) # was: (10, 0, 96)
#MESSAGE = "\"Space is big. Really big. You just won't believe how vastly hugely mind-bogglingly big it is. I mean, you may think it's a long way down the road to the chemist, but that's just peanuts to space.\" - Douglas Adams"
#MESSAGE = "              CT7AGR Paulus Schulinck, São Domingos de Benfica, Lisboa, Portugal. QTH Locator IM58js              "
MESSAGE = ''
HOLD_TIME = 0.5 # 1.0 # 2.0
STEP_TIME = 0.05 # s0.075


# constants for controlling the background colour throughout the day
MIDDAY_HUE = 1.1
MIDNIGHT_HUE = 0.8
HUE_OFFSET = -0.1

MIDDAY_SATURATION = 1.0
MIDNIGHT_SATURATION = 1.0

MIDDAY_VALUE = 0.8
MIDNIGHT_VALUE = 0.3

# create the rtc object
rtc = machine.RTC()

year, month, day, wd, hour, minute, second, _ = rtc.datetime()
last_second = second

# set up some pens to use later
WHITE = gr.create_pen(255, 255, 255)
BLACK = gr.create_pen(0, 0, 0)

btn_none = 0
btn_a = 1
btn_b = 2
btn_c = 4
btn_d = 8
btn_dict = {
    btn_none: "none",
    btn_a: "button a",
    btn_b: "button b",
    btn_c: "button c",
    btn_d: "button d"
    }

# state constants
STATE_PRE_SCROLL = 0
STATE_SCROLLING = 1
STATE_POST_SCROLL = 2

shift = 0
state = STATE_PRE_SCROLL

# set the font
gr.set_font("bitmap8")

# calculate the message width so scrolling can happen
msg_width = gr.measure_text(MESSAGE, 1)


ID = const(0)
LAT = const(1)
LATDIR = const(2)
LON = const(3)
LONDIR = const(4)
GS = const(5)
CRS = const(6)
DATE = const(7)
VAR = const(8)
VARDIR = const(9)
ALT = const(10)

class gps_msgs:

    def __init__(self):
        self.gps = ["",  "",  "",  "",  "", "", "", "", "", "", ""]

    def write(self, s):
        tp = isinstance(s,list)
        if tp == True:
            self.gps[ID] = s[ID] # ID
            self.gps[LAT] = s[LAT] # Lat
            self.gps[LATDIR] = s[LATDIR] # LadID N/S
            self.gps[LON] = s[LON] # Lon
            self.gps[LONDIR] = s[LONDIR] # LonID E/W
            self.gps[GS] = s[GS] # GS
            self.gps[CRS] = s[CRS] # CRS
            self.gps[DATE] = s[DATE] # Var
            self.gps[VAR] = s[VAR] # Var
            self.gps[VARDIR] = s[VARDIR] # VarDir
            self.gps[ALT] = s[ALT] # Alt

    def read(self, n):
        tp = isinstance(n, type(None))
        if tp == True:
            n = ID
        if n >= ID and n <= ALT:
            return self.gps[n]
        else:
            return self.gps

    def clean(self):
        self.gps[ID] = ""
        self.gps[LAT] = ""
        self.gps[LATDIR] = ""
        self.gps[LON] = ""
        self.gps[LONDIR] = ""
        self.gps[GS] = ""
        self.gps[CRS] = ""
        self.gps[DATE] = ""
        self.gps[VAR] = ""
        self.gps[VARDIR] = ""
        self.gps[ALT] = ""

my_msgs = gps_msgs()
if my_debug:
    print(f"global: type(my_msgs)= {type(my_msgs)}")

gc.collect()

last_time = time.ticks_ms()

def character(asc, xt, yt, r, g, b):  # Single character sz is size: 1 or 2
    colour = gr.create_pen(r, g, b)
    gr.set_pen(colour)
    code = asc * 5    # 5 bytes per character
    for ii in range(5):
        line = FONT[code + ii]
        for yy in range(8):
            if (line >> yy) & 0x1:
                gr.pixel(ii+xt, yy+yt) 
                                    
def prnt_st(asci, xx, yy, r, g, b):  # Text string
    move = 6
    for letter in(asci):
        asci = ord(letter)
        character(asci, xx, yy, r, g, b)
        xx = xx + move
        
def handle_rst(pin):
    if pin == button_rst:
        print("Going to reset...")
        gr.set_pen(gr.create_pen(0, 0, 0))
        gr.clear()
        prnt_st("Reset...", 6, 2, brill, 0, 0) # Text examples
        gu.update(gr)
        time.sleep(2)
        machine.reset()
        
def handle_a(pin):
    global curr_func, button_a_pressed
    if button_a_pressed:
        #button_a_pressed = False
        return # prevent handle bounce
    else:
        button_a_pressed = True
    le = len(func_dict)
    curr_func += 1
    if curr_func >= le:
        curr_func = 0
    print(f"handle_a(): new curr_func = {curr_func} (\'{func_dict[curr_func]}\')")

def handle_b(pin):
    global curr_func, button_b_pressed
    if button_b_pressed:
        #button_b_pressed = False
        return # prevent handle bounce
    else:
        button_b_pressed = True
    le = len(func_dict)
    curr_func -= 1
    if curr_func < 0:
        curr_func = le-1
    print(f"handle_b(): new curr_func = {curr_func} (\'{func_dict[curr_func]}\')")

def handle_c(pin):
    global button_c_pressed
    if button_c_pressed:
        return # prevent handle bounce
    else:
        button_c_pressed = True
    pass

def handle_d(pin):
    global button_d_pressed
    if button_d_pressed:
        return # prevent handle bounce
    else:
        button_d_pressed = True
    pass

def clr_buttons():
    global button_a_pressed, button_b_pressed, button_c_pressed, button_d_pressed
    button_a_pressed = False
    button_b_pressed = False
    button_c_pressed = False
    button_d_pressed = False

def handle_lux_up(pin):
     gu.adjust_brightness(+0.01)

def handle_lux_dn(pin):
    gu.adjust_brightness(-0.01)
    pass

# We use the IRQ method to detect the button presses to avoid incrementing/decrementing
# multiple times when the button is held.
button_a.irq(trigger=machine.Pin.IRQ_FALLING, handler=handle_a)
button_b.irq(trigger=machine.Pin.IRQ_FALLING, handler=handle_b)
button_c.irq(trigger=machine.Pin.IRQ_FALLING, handler=handle_c)
button_d.irq(trigger=machine.Pin.IRQ_FALLING, handler=handle_d)
if use_sound:
    button_vol_up.irq(trigger=machine.Pin.IRQ_FALLING, handler=handle_vol_up)
    button_vol_dn.irq(trigger=machine.Pin.IRQ_FALLING, handler=handle_vol_dn)
button_lux_up.irq(trigger=machine.Pin.IRQ_FALLING, handler=handle_lux_up)
button_lux_dn.irq(trigger=machine.Pin.IRQ_FALLING, handler=handle_lux_dn)
button_rst.irq(trigger=machine.Pin.IRQ_FALLING, handler=handle_rst)

if use_sound:
    timer = machine.Timer(-1)

    # The two frequencies to play
    tone_a = 1000
    tone_b = 900
    vol = 10  # initially no sound, only after user presses 'Vol +'
    min_vol = 10
    max_vol = 20000
    vol_step = 50
    

    notes = [(1000, 900), (1000, 900)]
    channels = [gu.synth_channel(i) for i in range(len(notes))]
    
    def handle_vol_up(pin):
        global vol
        if vol > 0:  # Zero means tone not playing
            # Increase Tone A
            vol = min(vol + 10, max_vol)
            #channels[0].frequency(vol)
            text = "Vol Up"+' '+str(vol)
            gr.set_pen(gr.create_pen(0, 0, 0))
            gr.clear()
            outline_text(text)

    def handle_vol_dn(pin):
        global vol
        if vol > 0:  # Zero means tone not playing
            # Decrease Tone A
            vol = max(vol - 10, min_vol)
            #channels[0].frequency(vol)
            text = "Vol Dn"+' '+str(vol)
            gr.set_pen(gr.create_pen(0, 0, 0))
            gr.clear()
            outline_text(text, cnt=0)

    def play_tone(tone):
        global vol
        if vol <= 10:
            return  # don't make sound
        if tone >= 0 and tone <=1000:
            # Stop synth (if running) and play Tone A
            timer.deinit()
            if tone == tone_a:
                channels[0].play_tone(tone, 0.06)
            if tone == tone_b:
                channels[1].play_tone(tone_b, 0.06, attack=0.5)

            gu.play_synth()

    def double_tone():
        global tone_a, tone_b
        TAG="double_tone(): "
        tone_a = 1000
        tone_b = 900
        ch_a = 0
        ch_b = 1
        tone = None
        ch = None

        for _ in range(2):
            timer.deinit() 
            tone = tone_a if _ == 0 else tone_b
            # print(TAG+f"playing tone {tone}")
            ch = ch_a if _ == 0 else ch_b
            if tone > 0:  # Zero means tone not playing
                play_tone(tone)
                time.sleep(0.3)
        gu.stop_playing()
        timer.deinit()
        tone_a = 0
        tone_b = 0
#---------------------------------------------------------------------------
def scroll_text(msg, do_scroll):
    global state, PADDING, BACKGROUND_COLOUR, shift
    # set the font
    TAG="scroll_text(): "
    gr.set_font("bitmap6")  #"bitmap8")

    # calculate the message width so scrolling can happen
    msg_width = gr.measure_text(msg, 1)

    last_time = time.ticks_ms()

    cnt = 0
    
    print(TAG+f"going to scroll text: \'{msg}\'")
    while True:
        time_ms = time.ticks_ms()

        if gu.is_pressed(gu.SWITCH_BRIGHTNESS_UP):
            gu.adjust_brightness(+0.01)

        if gu.is_pressed(gu.SWITCH_BRIGHTNESS_DOWN):
            gu.adjust_brightness(-0.01)

        if do_scroll:
            if state == STATE_PRE_SCROLL and time_ms - last_time > HOLD_TIME * 1000:
                if msg_width + PADDING * 2 >= width:
                    state = STATE_SCROLLING
                last_time = time_ms

            if state == STATE_SCROLLING and time_ms - last_time > STEP_TIME * 1000:
                shift += 1
                if shift >= (msg_width + PADDING * 2) - width - 1:
                    state = STATE_POST_SCROLL
                last_time = time_ms

            if state == STATE_POST_SCROLL and time_ms - last_time > HOLD_TIME * 1000:
                state = STATE_PRE_SCROLL
                shift = 0
                last_time = time_ms
        else:
            PADDING = 2
            
        gr.set_pen(gr.create_pen(int(BACKGROUND_COLOUR[0]), int(BACKGROUND_COLOUR[1]), int(BACKGROUND_COLOUR[2])))
        gr.clear()

        #outline_text(msg, x=PADDING - shift, y=2)
        outline_text(msg, x=PADDING - shift, y=2, cnt=cnt)

        # update the display
        gu.update(gr)

        # pause for a moment (important or the USB serial device will fail)
        time.sleep(0.001)
        cnt += 1
        if cnt >= msg_width:
            break

#@micropython.native  # noqa: F821
def from_hsv(h, s, v):
    i = math.floor(h * 6.0)
    f = h * 6.0 - i
    v *= 255.0
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t = v * (1.0 - (1.0 - f) * s)

    i = int(i) % 6
    if i == 0:
        return int(v), int(t), int(p)
    if i == 1:
        return int(q), int(v), int(p)
    if i == 2:
        return int(p), int(v), int(t)
    if i == 3:
        return int(p), int(q), int(v)
    if i == 4:
        return int(t), int(p), int(v)
    if i == 5:
        return int(v), int(p), int(q)

phase = 0
hue_map = [from_hsv(x / width, 1.0, 1.0) for x in range(width)]
hue_offset = 0.0
stripe_width = 3.0
speed = 5.0

# function for drawing a gradient background
def gradient_background(start_hue, start_sat, start_val, end_hue, end_sat, end_val):
    half_width = width // 2
    for x in range(0, half_width):
        hue = ((end_hue - start_hue) * (x / half_width)) + start_hue
        sat = ((end_sat - start_sat) * (x / half_width)) + start_sat
        val = ((end_val - start_val) * (x / half_width)) + start_val
        colour = from_hsv(hue, sat, val)
        gr.set_pen(gr.create_pen(int(colour[0]), int(colour[1]), int(colour[2])))
        for y in range(0, height):
            gr.pixel(x, y)
            gr.pixel(width - x - 1, y)

    colour = from_hsv(end_hue, end_sat, end_val)
    gr.set_pen(gr.create_pen(int(colour[0]), int(colour[1]), int(colour[2])))
    for y in range(0, height):
        gr.pixel(half_width, y)

# function for drawing outlined text
def outline_text(text, x, y, alt_clr=(255,255,255), cnt=0):
    #gr.set_font("bitmap6")  #"bitmap8")

    if x < 0:
        x = -x  # make it positive
    if not my_debug and cnt == 0:
        print(f"outline_text(): text = \'{text}\', x,y = {x},{y}")
    #gr.set_pen(gr.create_pen(alt_clr[0], alt_clr[1], alt_clr[2]))
    gr.set_pen(WHITE)
    gr.text(text, x, y, -1, 1)

def pressed():
    if gu.is_pressed(gu.SWITCH_A):
        return gu.SWITCH_A
    if gu.is_pressed(gu.SWITCH_B):
        return gu.SWITCH_B
    if gu.is_pressed(gu.SWITCH_C):
        return gu.SWITCH_C
    if gu.is_pressed(gu.SWITCH_D):
        return gu.SWITCH_D
    return None

def ck_button():
    if pressed() == gu.SWITCH_A:
        return btn_a
    elif pressed() == gu.SWITCH_B:
        return btn_b
    elif pressed() == gu.SWITCH_C:
        return btn_c
    elif pressed() == gu.SWITCH_D:
        return btn_d
    else:
        return 0
    
# wait until all buttons are released
while pressed() != None:
    time.sleep(0.1)
  
# Connect to wifi and synchronize the RTC time from NTP
def sync_time(show):
    if not wifi_available:
        return

    TAG="sync_time(): "
    # Start connection
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    # Wait for connect success or failure
    max_wait = 100
    msg_shown = False
    while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        max_wait -= 1
        if not msg_shown:
            msg_shown = True
            print(TAG+'waiting for connection...')
        time.sleep(0.2)

        if show:
            redraw_display_if_reqd()
            gu.update(gr)

    if max_wait > 0:
        print(TAG+"Connected")

        try:
            ntptime.host = NTP_SERVER
            ntptime.settime()
            print(TAG+"Time set")
        except OSError:
            pass

    wlan.disconnect()
    wlan.active(False)
    
# NTP synchronizes the time to UTC, this allows you to adjust the displayed time
# by one hour increments from UTC by pressing the volume up/down buttons
#
# We use the IRQ method to detect the button presses to avoid incrementing/decrementing
# multiple times when the button is held.
utc_offset = TZ_OFFSET

#
# return a quasi unix epoch value
# to be used in main() to calculate the elapsed time in seconds
# 
def epoch():
    secs = time.time() + (utc_offset * 3600)
    if my_debug:
        print(f"epoch(): seconds= {secs}")
    return secs

# Check whether the RTC time has changed and if so redraw the display
def redraw_display_if_reqd():
    global year, month, day, wd, hour, minute, second, last_second

    tm = time.time() + (utc_offset * 3600)
    tm_local = time.localtime(tm)
    if my_debug:
        print(f"redraw_display_if_reqd(): tm_local= {tm_local}")
    year   = tm_local[0]
    month  = tm_local[1]
    day    = tm_local[2]
    hour   = tm_local[3]
    minute = tm_local[4]
    second = tm_local[5]
    wd     = tm_local[6]
    #yd     = tm_local[7]
    if second != last_second:
        hour += utc_offset
        time_through_day = (((hour * 60) + minute) * 60) + second
        percent_through_day = time_through_day / 86400
        percent_to_midday = 1.0 - ((math.cos(percent_through_day * math.pi * 2) + 1) / 2)
        print(percent_to_midday)

        hue = ((MIDDAY_HUE - MIDNIGHT_HUE) * percent_to_midday) + MIDNIGHT_HUE
        sat = ((MIDDAY_SATURATION - MIDNIGHT_SATURATION) * percent_to_midday) + MIDNIGHT_SATURATION
        val = ((MIDDAY_VALUE - MIDNIGHT_VALUE) * percent_to_midday) + MIDNIGHT_VALUE

        gradient_background(hue, sat, val,
                            hue + HUE_OFFSET, sat, val)

        clock = "{:02}:{:02}:{:02}".format(hour, minute, second)

        # set the font
        gr.set_font("bitmap8")

        # calculate text position so that it is centred
        w = gr.measure_text(clock, 1)
        x = int(width / 2 - w / 2 + 1)
        y = 2

        outline_text(clock, x, y, cnt=0)

        last_second = second


gu.set_brightness(0.2)  # was: (0.5)

class HdgRibbon():
    def __init__(self):
        global width, height
        if my_debug:
            print("HdgRibbon.__init__(): we passed here")
        self.clr = 155
        self.fs_heading = 0
        self.heading_old = 0.0
        self.fs_hdg_step = None
        self.x = 0  # Left  was:25
        # self.y = HEIGHT / 2
        self.y = height / 2
        self.next_y = self.y
        # self.width = WIDTH
        self.width = width
        #self.height = HEIGHT
        self.height = height
        self.digit_width = 8   # for the Pimoroni HUB75 LED matrix panels
        self.digit_height = 15 # idem
        self.digit_spacing = 2
        self.ox = self.width // 2
        self.oy = self.height // 2
        self.hue = 0
        # self.h_pts =   [  0,  12,  22,  32,  42,  52,  64]   # for the HUB75 LED matrix panels
        self.h_pts =   [  2,  10,  18,  26,  34,  42,  50]   # for the Galactic Unicorn LED matrix panel
        self.hdg_pts = [-30, -20, -10,   0,  10,  20,  30]
        sleep(1)
        
        
    def ribbon_base(self):
        TAG = "ribbon.ribbon_base(): "
        r = 0   # int(OUTLINE_COLOUR[0])
        g = 100 # int(OUTLINE_COLOUR[1])
        b = 100 # int(OUTLINE_COLOUR[2])
        r1 = 100
        g1 = 0
        b1 = 0
        gr.set_pen(gr.create_pen(0, 0, 0))
        gr.clear()
        #------------------------------------------
        # Draw the base line
        #------------------------------------------
        gr.set_pen(gr.create_pen(r1, g1, b1))
        # print(TAG+f"outline colours: {r1},{g1},{b1}")
        for i in range(len(self.h_pts)):
            x = self.h_pts[i]
            y1 = self.height-2
            y2 = self.height-1
            # print(TAG+f"x,y1,y2 = {x},{y1},{y2}")
            gr.pixel(self.h_pts[i], self.height-2)
        for i in range(width):
            gr.pixel(i, self.height-1)
        #-------------------------------------------
        # Calculate
        #-------------------------------------------
        #gr.set_pen(gr.create_pen(r, g, b))
        h = self.fs_heading
        h_lst = []
        hi_lst = []
        w = 0
        for _ in range(3):
            if _ == 0:
                h_dev = -2  # Previous heading value
            elif _ == 1:
                h_dev = 0  # Current heading value
            elif _ == 2:
                h_dev = 2  # Next heading value
            h_lst.append( "{:03d}".format(int(round(float(h + h_dev)))) )   # hdg-1   hdg   hdg+1
            hi_lst.append( h + h_dev)
        s_width = gr.measure_text(h_lst[1], 1) // 2 # calculate the width of the middle item
        if my_debug:
            print(TAG+f"s_width = {s_width}")
        #-------------------------------------------
        # Draw the heading values
        #-------------------------------------------
        bg = (0, 0, 0)
        for _ in range(1, 6, 2):
            # print(TAG+f"_ = {_}")
            if _ == 1:
                idx = 0
                y = 0
                fg = (r, g, b)
                """
                print(TAG+f"h_lst[idx][:1] = \'{h_lst[idx][:1]}\'")
                if h_lst[idx][:1] == "0":
                    x_comp = -3
                else:
                    x_comp = 0
                """
                x = 1
            elif _ == 3:
                idx = 1
                y = 2
                fg = (r1, g1, b1)
                """
                if h_lst[idx][:1]  == "0":
                    x_comp = 0
                else:
                    x_comp = 1
                """
                x = 19
            elif _ == 5:
                idx = 2
                y = 0
                fg = (r, g, b)
                """
                if h_lst[idx][:1]  == "0":
                    x_comp = 2
                else:
                    x_comp = 2
                """
                x = 36
            gc.collect()
            gr.set_pen(gr.create_pen(fg[0], fg[1], fg[2]))
            #outline_text(h_lst[idx], self.h_pts[_] - s_width + x_comp, y, fg, cnt=0)
            #outline_text(h_lst[idx], x, y, fg, cnt=0)
            gr.text(h_lst[idx], x, y, -1, 1)

        gu.update(gr)

    # Wrapper function  - called from main()
    def draw_number(self, x, y, number, fg=None, bg=None):
        scroll_text(number, False)

    def set_heading_fm_sim(self, hdg):  # called by disp_crs()
        global my_debug
        if my_debug:
            print("set_heading_fm_sim(): heading set to: ", hdg)
        self.fs_heading = hdg


# --------------------------------------------------  +
# Prototypes of the 13 functions in this script file: |
# --------------------------------------------------  +
"""
    def ck_uart(): # (nr_bytes)
    def disp_crs(): # ()
    def empty_buffer():  # (void)
    def led_toggle(): # (void)
    def reset():  # (void)
    def ck_gs(): # (float)
    def ac_is_stopped(void) # (bool)
    def ac_is_taxying(show_speed) # (bool)
    def disp_crs() # (void)
    def disp_pos() # (void)
    def disp_gs() # (void)
    def disp_alt() # (void)
    def loop(): # (void)
    def main():
"""

# From: ProS3_nr_1/2022-10-08_Github_new_repo_sercom_I2C/2022-10-12_16h46_Ver2_Sensor_backup/code_current.py
def reset():
    print("Going to reset.\nHang on...")
    machine.reset()

def ck_gs():
    global v_gs, my_msgs
    TAG= "ck_gs(): "
    t_gs = my_msgs.read(GS)
    le = len(t_gs)
    if my_debug:
        print(TAG,"value of gs = {}, len(gs) = {}".format(t_gs, le), end='\n')
    if le == 0: # check for t_gs == "". This happened sometimes!
        v_gs = 0
    else:
        v_gs = round(int(float(t_gs)))
    if not my_debug:
        print(TAG,"value of v_gs = {}".format(v_gs), end='\n')
    return v_gs

def nodata():
    global ac_stat
    TAG= "nodata(): "
    s = "no data"
    if ac_stat != ac_no_data:
        ac_stat = ac_no_data
    scroll_text(s, False)
    print(TAG+s)
    
def ac_status():
    global ac_stat, v_gs
    TAG= "ac_status(): "
    stats_dict = {
        ac_no_data: "no data",
        ac_stopped: "parked",
        ac_taxying: "taxying",
        ac_flying: "flying"
    }
    v_gs = ck_gs()
    if v_gs < 0.2:
        ac_stat = ac_stopped
    elif v_gs >= 0.2 and v_gs <= 30:  # keep margin. Sometimes while parked the gs can be 0.1
        ac_stat = ac_taxying
    elif v_gs > 30:
        ac_stat = ac_flying
    if not my_debug:
        if ac_stat >= ac_no_data and ac_stat <= ac_flying:
            print(TAG+f"airplane is {stats_dict[ac_stat]}")
   
"""
 Function copied from: I:\pico\paul_projects\pico\circuitpython\msfs2020_gps_rx_picolipo\2021-09-03_16h49_ver
"""

def ac_is_stopped():
    global ac_stat 
    TAG = "ac_is_stopped(): "
    s = "Aircraft is stopped or parked"
    t_elapsed = 0
    lelapsed = True
    if ac_stat == ac_stopped:
        scroll_text("ac parked", False)
        print(s, end = '\n') # Alway print to REPL (it does almost immediately)


def ac_is_taxying(show_speed=False):
    global ac_stat
    TAG= "ac_is_taxying(): "
    s = "Airplane is taxying"
    if ac_stat == ac_taxying: 
        if not show_speed:
            s = "taxying"
        else:
            s = 'Speed {} kts'.format(v_gs)    
        scroll_text(s, False)
        print(TAG+s)


def loop():
    global startup, led, lp_cnt, ID_s, lstop, previousMillis, led_interval, biLdIsOn, gs_old, \
    biLdIsOn, msg_nr, rx_buffer, msg_lst, nr_msg_items, width, old_func


    TAG = "loop(): "
    chrs_rcvd = 0  # (int) holds the number of characters received via the serial communication line
    c = 0  # (unsigned char)
    
    gs = 0.0
    gs_old = 0.0
    msg_rx_ok = 0

    # currentMillis (See Adafruit: 'time in CircuitPython')
    currentMillis = ticks_ms()
    s = "board " + sys.platform+" " # for the Pi Pico the value is: 'RP2040'
    gr.set_pen(WHITE)
    outline_text(s, x=3, y=2, cnt=0)
    print(s)
    sleep_for = 5
    time.sleep(3)
    led_toggle()
    biLdIsOn = False 
    dh_row = 0
    font_size = 1  # heading font size

    print()
    print("MSFS2020 GPS GPRMC data reception decoder sketch by Paulsk (mailto: ct7agr@live.com.pt). ")
    print("\nNumber of loops in this run: {}".format(max_lp_cnt))
    sleep(0.5)  # <=================================== DELAY ===============================<
    """
    # Collect Serial Input NON-BLocking.
    # Continue receiving characters until '\n' (linefeed) is received.
    """
    split_err = False
    chrs_rcvd = 0  # reset the nr of characters received
    print("........................", end="\n")
    msg_shown = False
    while True:
        try:
            lp_cnt += 1  # increase the loop counter
            ID_s = ''  # Clear the ID string
            # lcd.clear()  # clean the LCD
            gr.clear()
            print("\nStart of loop {}".format(lp_cnt), end="\n")
            if startup == -1 and not msg_shown:
                msg_shown = True
                scroll_text("RX msgs...", False) #, x=1 - shift, y=2)
                sleep(sleep_for)
                gr.set_pen(gr.create_pen(0, 0, 0))
                gr.clear()
                gu.update(gr)
            wait_cnt = 0
            # +--------------- RECEPTION ----------------------------------------+
            chrs_rcvd = ck_uart()  # read a complete GPS GPRMC datagram sentence |
            # +------------------------------------------------------------------+
            sleep(0.02) # just a little pause to avoid entry of zeros  # <==================== DELAY =======================================<
            if chrs_rcvd == -1:
                raise KeyboardInterrupt
            if chrs_rcvd > 0:
                if not my_debug:
                    print(TAG+f"Msg nr: {msg_nr}, ID: {ID_s}, nr characters rcvd from ck_uart() is: {chrs_rcvd}")
                    #print(TAG+"GPS data character received: ")
                    print(f"{rx_buffer.decode('utf-8')}")
                    #  print the rx_buffer less the \r\n at the end
                    #print(TAG+"Msg nr: {}, ID: {}, characters rcvd from ck_uart() is: {}, contents: \n\"{}\"".format(msg_nr,
                    #    ID_s, chrs_rcvd, rx_buffer[:-2]), file=sys.stderr)
                lResult = add_data()
                print(TAG+"add_data() result = {}".format(lResult))
                if lResult:
                    ac_status() # Get the airplane's status: no_data, stopped, taxying or flying
                    msg_rx_ok += 1
                    if ac_stat == ac_stopped:
                        ac_is_stopped()
                    elif ac_stat == ac_taxying:
                        ac_is_taxying(False)
                        #startup = 0
                    elif ac_stat == ac_flying:
                        if startup == -1:
                            gr.clear()
                        time.sleep(0.1)  # give a break to handle interrupts
                        #led_toggle()  # we toggle elsewhere (when receiving msg in ck_uart() )
                        if func_dict[curr_func] == "crs_func":
                            if not disp_crs():
                                return False
                        if func_dict[curr_func] == "pos_func":
                            if not disp_pos():
                                return False
                        if func_dict[curr_func] == "gs_func":
                            if not disp_gs():
                                return False
                        if func_dict[curr_func] == "alt_func":
                            if not disp_alt():
                                return False
                        if old_func != curr_func:
                            old_func = curr_func
                            clr_buttons()
                        gc.collect()

                        startup = 0
                    else:
                        pass
                else:
                    split_err = True
                if split_err == True:
                    print(TAG+"Error: spliting rx_buffer contents has failed. Exitting...", end="\n")
                    split_err = False
                # Do cleanup and resets
                chrs_rcvd = 0
                empty_buffer()
                #sleep(0.1)  # <============================== DELAY ===============================<
                if msg_nr >= max_lp_cnt:
                    #lstop = True  # force end of sketch execution
                    pass
                if lstop == True:
                    # print("\n\nWe go to stop")
                    if biLdIsOn:
                        led_toggle()
                else:
                    if startup == -1:
                        print("Waiting for serial com line to become available...")
                        startup = 0  # switch off flag. Showing this text only once.
                if (currentMillis - previousMillis) >= led_interval:
                    previousMillis = currentMillis
                print("End of loop {}".format(lp_cnt), end="\n")
                print("........................", end="\n")
                if msg_nr >= max_lp_cnt:
                    msg_nr = 0
                    lp_cnt = 0
            # End if (chrs_rcvd > 0)
            else:
                nodata()
                # No GPS sentences data received. Exit...
                pass
            if lstop == True:
                break
        except KeyboardInterrupt:
            return False
        except MemoryError:
            gc.collect()
    # end-of while True

    sleep(0.5)  # <======================================= DELAY ==============================<
    return True
# End of setUp()

"""
ck_uart(void) -> nr_bytes
        This function attempt to read the uart. It filters isolated \x00 byte characters
        Parameters: None
        Return: nr_bytes
"""
def ck_uart():
    global rx_buffer, msg_nr, loop_time, nRMC, nGGA, GPRMC_lst, GPGGA_lst, le_GPRMC_lst, le_GPGGA_lst
    TAG = 'ck_uart(): '
    nr_bytes = i = 0
    delay_ms = 0.3
    nGPRMC = -1
    nGPGGA = -1
    GPRMC_msg = None
    GPGGA_msg = None
    GPRMC_lst = []
    GPGGA_lst = []
    le_GPRMC_lst = 0
    le_GPGGA_lst = 0
    GPRMC_done = False
    GPGGA_done = False
    i = 0
    while True:
        try:
            gc.collect()
            rx_buffer = bytearray(rx_buffer_len * b'\x00')
            #nr_bytes = uart.readinto(rx_buffer)
            rx_buffer = uart.readline()
            if rx_buffer is not None:
                nr_bytes = len(rx_buffer)
            else:
                i += 1
                if i > 1000:
                    return 0  # Exit
                if i > 0 and i % 100 == 0:
                    nodata()
                time.sleep(delay_ms)
                continue
            if nr_bytes is not None:
                if my_debug:
                    print(TAG+"(just after uart.readinto(rx_buffer) nr of bytes= ", nr_bytes)
            #for i in range(5):
            #    time.sleep(delay_ms)
            loop_time = time.ticks_ms()  # time.time_ns()
            if nr_bytes is None:
                if my_debug:
                    print(TAG+"nr_bytes is None")
                time.sleep(delay_ms)
                continue
            if not nr_bytes:
                if my_debug:
                    print(TAG+"nr_bytes=", nr_bytes)
                time.sleep(delay_ms)
                continue
            if nr_bytes > 1:
                try:
                    rx_buffer_s = rx_buffer.decode('utf-8')
                except UnicodeError:  # Happens mostly if serial connection is broken
                    print(TAG+"Check serial wiring")
                    time.sleep(delay_ms)
                    continue
                gc.collect()
                if not GPRMC_done:
                    nGPRMC = rx_buffer_s.find("$GPRMC")
                    if nGPRMC >= 0:
                        n = rx_buffer_s.find("$GPGGA")
                        if n >= 0:
                            GPRMC_msg = rx_buffer_s[:nGPGGA]
                        else:
                            GPRMC_msg = rx_buffer_s
                    else:
                        time.sleep(delay_ms)
                        continue

                    GPRMC_lst =  GPRMC_msg.split(",")
                    le_GPRMC_lst = len(GPRMC_lst)
                    if my_debug:
                        print(TAG+f"type(GPRMC_msg)= {type(GPRMC_msg)}")
                        print(TAG+f"GPGRMC msg = {GPRMC_msg}")
                    if le_GPRMC_lst >= 12:  # A complete GPRMC msg is usually 78 characters
                        if my_debug:
                            print(TAG+f"GPRMC_msg is complete: {GPRMC_msg}. len(GPRMC_lst)= {le_GPRMC_lst}")
                        GPRMC_done = True
                    else:
                        time.sleep(delay_ms)
                        continue   
                if not GPGGA_done:
                    nGPGGA = rx_buffer_s.find("$GPGGA")
                    if nGPGGA >= 0:
                        # check if $GPRMC occurs near the end of the GPGGA message:
                        n = rx_buffer_s.find("$GPRMC")
                        if n > 0:
                            GPGGA_msg = rx_buffer_s[nGPGGA:n]
                        else:
                            GPGGA_msg = rx_buffer_s[nGPGGA:]
                        n = GPGGA_msg.find('\r\n')
                        if n < 0:
                            GPGGA_msg += '\r\n'
                        GPGGA_lst = GPGGA_msg.split(",")
                        le_GPGGA_lst = len(GPGGA_lst)
                        if le_GPGGA_lst >= 15:  # A complete GPGGA msg is usually 78 characters
                            if my_debug:
                                print(TAG+f"GPGGA_msg is complete: {GPGGA_msg}. len(GPGGA_lst)= {le_GPGGA_lst}")
                            GPGGA_done = True
                        else:
                            time.sleep(delay_ms)
                            continue
                    else:
                        time.sleep(delay_ms)
                        continue              
                        
                if GPRMC_done and GPGGA_done:
                    rx_buffer_s = GPRMC_msg + GPGGA_msg
                    nr_bytes = len(rx_buffer_s)
                    rx_buffer = rx_buffer_s.encode('utf-8')
                    if my_debug:
                        print(TAG+"GPRMC_msg + GPGGA_msg = {rx_buffer}")
                    break

                if not my_debug:
                    print(TAG+"nr of bytes= ", nr_bytes)
                if nr_bytes < 120: # was 146 Try to cut short checking. A full/OK msg has 153 characters
                    time.sleep(delay_ms)
                    continue
                print(TAG+"rcvd data: {}".format(rx_buffer),end="\n")
                # Check for '*' in tail. If present, the message is very probably complete.
                tail_part = -20
                tp = rx_buffer[tail_part:].decode('utf-8')
                print(TAG+f"rx_buffer[{tail_part}:]= \'{tp}\'")
                n1 = tp.find('*') # find '*' in tail 10 characters. If not found, loop
                #n1 = rx_buffer.find('*')
                print(TAG+f"n1 = {n1}")
                if n1 < 0: # no '*' in tail
                    time.sleep(delay_ms)
                    continue
                else:
                    if nGPRMC < 0:
                        n2 = rx_buffer_s.find('$GPRMC')
                    else:
                        n2 = nGPRMC
                    # print(TAG+f"n2 = {n2}")
                    if n2 < 0:
                        time.sleep(delay_ms)
                        continue
                    else:
                        nRMC = n2
                    if nGPGGA < 0:
                        n3 = rx_buffer_s.find('$GPGGA')
                    else:
                        n3 = nGPGGA
                    # print(TAG+f"n3 = {n3}")
                    if n3 < 0:
                        time.sleep(delay_ms)
                        continue
                    else:
                        nGGA = n3
                    n4 = int(nr_bytes*0.75) # get 3/4 of length of characters received
                    # print(TAG+f"n4 = {n4}")
                    if not my_debug:
                        #print(TAG+"n1 = {}, n2 = {}, n3= {}, n4 ={}".format(n1, n2, n3, n4))
                        print(TAG+"* at {}, $GPRMC at {}, $GPGGA at {}, n4 ={}".format(n1+tail_part+1, n2, n3, n4))
                    if n2 > n4: # check if $GPRMC occurs at more than 3/4 of the characters received
                        time.sleep(delay_ms)
                        continue
                    else:
                        # print(TAG+"returning with {} nr of bytes".format(nr_bytes))
                        #return nr_bytes
                        break
            elif nr_bytes == 1:
                if rx_buffer[0] == b'\x00':
                    time.sleep(delay_ms)
                    i += 1
                    if i % 1000 == 0:
                        print("Waiting for uart line to become ready")
            else:
                empty_buffer()
                time.sleep(delay_ms)
                continue
        except KeyboardInterrupt:
            nr_bytes = -1
    return nr_bytes


"""
add_data(void) -> lResult
        This function collects and writes the received gps data to the my_msgs object.
        It truncates the variation longitudinal indicator letter in the last item
        (a combination of magnetic variation longitudinal indicator and a 2-character checksum).
        It moves the check_sum characters into a 12th element to this list and
        Parameters: None
        Return: le2 (i.e.: len(msg_lst)). If it fails to split, a value False will be returned

"""
def add_data():
    global my_msgs, GPRMC_lst, GPGGA_lst, le_GPRMC_lst, le_GPGGA_lst
    TAG = "add_data(): "
    lResult = True
    lGPRMC_go = lGPGGA_go = False

    if le_GPRMC_lst == 12 and le_GPGGA_lst == 15:
        if not my_debug:
            print(TAG+f"We're using GPS data from rcvd GPRMC and GPGGA msgs.\nNr GPRMC items= {le_GPRMC_lst}. Nr of GPGGA items= {le_GPGGA_lst}")
        rmc_lst = [
            GPRMC_lst[0],   # id
            GPRMC_lst[3],   # lat
            GPRMC_lst[4],   # latdir
            GPRMC_lst[5],   # lon
            GPRMC_lst[6],   # londir
            GPRMC_lst[7],   # gs
            GPRMC_lst[8],   # track true
            GPRMC_lst[9],   # date
            GPRMC_lst[10],  # var
            GPRMC_lst[11]   # vardir
        ]
        if le_GPRMC_lst == 12:
            lGPRMC_go = True
            GPRMC_lst[11] = GPRMC_lst[11][:1]  # extract 'E' or 'W' from (e.g.:) 'E*72'
            
        if le_GPGGA_lst == 15:
            if my_debug:
                print(TAG+f"type(GPGGA_lst[9])= {type(GPGGA_lst[9])}, value= {GPGGA_lst[9]}")
            t_alt = float(GPGGA_lst[9])
            if my_debug:
                print(TAG+f"t_alt = {t_alt}, type(t_alt)= {type(t_alt)}")
            p = isinstance(t_alt,float)
            if p == True:
                #                 alt
                t_alt = round(int(t_alt) * 3.2808)  # convert meters into feet
            else:
                t_alt = 0
            lGPGGA_go = True
            
        if lGPGGA_go == True:
            rmc_lst.append(str(t_alt))
        else:
            rmc_lst.append("0")
        
        my_msgs.write(rmc_lst)
    

        if my_debug:
            print(TAG+"cross-check: my_msgs class data contents: {}".format(my_msgs.read(ALT)), end="\n")
    
    if lGPRMC_go == False and lGPGGA_go == False:
        lResult = False

    return lResult

# funct time_elapsed
# param t1 in nanosecond (derived from time.ticks_ms())
# param t2 in nanosecond # same
# return: difference of t2 and t1 + rounding compensation * division factor to change into milliseconds
def time_elapsed(t1, t2):
    # t_corr = 500000
    # t_make_ms = 1000000
    if t2 >= t1:
        t3 = t2 - t1
    else:
        t3 = t1 - t2
    return t3  # (t3 + t_corr) // t_make_ms


"""
led_toggle(void) -> void
        This function toggles the built-in led
        Sets/resets the global variable biLdIsOn
        Parameters: None
        Return: None
"""
def led_toggle():
    global biLdIsOn, led

    if biLdIsOn:
        #led.set_rgb(0, 0, 0)
        biLdIsOn = False
    else:
        #led.set_rgb(0, 50, 0)
        biLdIsOn = True       

"""
empty_buffer(void) -> void
        This function clears the rx_buffer
        Parameters: None
        Return: None
"""
def empty_buffer():
    global rx_buffer
    rx_buffer = bytearray(rx_buffer_len * b'\x00')


"""
disp_var(void) -> void
        This function displays the variation
        Parameters: None
        Return: None
"""    
def disp_var():
    TAG = "disp_var(): "
    var_val = float(my_msgs.read(VAR))  # floating point value
    print(TAG+f"var_val: {var_val}", end='\n')
    var_dir = my_msgs.read(VARDIR)
    tmg_true = get_mag()
    gr.clear()
    if var_dir == "E":
        s1 = "-"
    elif var_dir == "W":
        s1 = '+'
    s2 = "var {:s}{:2d}".format(s1, var_val)
    scroll_text(s2, False)
    time.sleep(3)

def get_mag():
    var_val = float(my_msgs.read(VAR))  # floating point value
    var_dir = my_msgs.read(VARDIR)
    tmg_true = float(my_msgs.read(CRS))
    if var_dir == "E":   # Correct for variation
        trk_mag = tmg_true - var_val  # NOTE !!! this is the opposite calculation than from magnetic +- variation to true heading
    elif var_dir == "W":
        trk_mag = tmg_true + var_val  # idem
    return trk_mag

"""
mag_or_tru(void) -> boolean
        This function discerns if True or Magnetic track will be used
        Depends on the latitude of the airplane's position 
        Parameters: None
        Return: boolean: True if Magnetic. False if True
        
        Source: https://www.navcanada.ca/en/changing-from-magnetic-to-true-tracks-in-aviation.pdf
        At latitudes above about 60° tracks and routes published on charts are
        given in True because of the weakness of the horizontal component of the magnetic field and
        because it changes so rapidly with both location and time. 
        
        Next info added by @Paulskpt. Also decision to use True in the Southern hemisphere from and above latitude 40°
        In 2020 position of the magnetic poles:
        Nortpole: 86.494°N 162.867°E
        Southpole: 64.081°S 135.866°E
"""
def mag_or_tru():
    global lMagnetic, lTrackDirChgd
    lat = float(my_msgs.read(LAT))
    latdir = my_msgs.read(LATDIR)
    lMag = True
        
    # Note: based on the above mentioned document:
    if latdir == "N":
        if lat >= 60.0:
            lMag = False  # True
    elif latdir == "S":
        if lat >= 40.0:
            lMag = False # True
    if lMag != lMagnetic:
        lMagnetic = lMag
        lTrackDirChgd = True
    return lMag

def disp_crs():
    global GPRMC_cnt, loop_time, biLdIsOn, lTrackDirChgd
 
    try:
        TAG = "disp_crs(): "
 
        lDispMagOrTru = mag_or_tru()
    
        #var_val = float(msg_lst[9])  # floating point value
        var_val = float(my_msgs.read(VAR))  # floating point value
        print(TAG+f"var_val: {var_val}", end='\n')
        #var_dir = msg_lst[10]
        var_dir = my_msgs.read(VARDIR)
        trk_mag =  float(0.0)  # floating point value
        tmg_true = float(0.0)  # track made good true (floating point value)
        s_tmg_true = ""
        # $GPRMC message parts detail indexes:
        tmg =  CRS # index to msg_lst. type float  (e.g.: 338.1)     < used

        if my_debug:
            print(TAG+"GPRMC_cnt: {}".format(GPRMC_cnt), end="\n")

        if my_debug:
            msg_lst = my_msgs.read(ALT+1)  # force to receive a list
            print(TAG, end='')
            le = len(msg_lst)
            if le > 0:
                for _ in range(len(msg_lst)):
                    print("{}, ".format(msg_lst[_]), end='')
                print()
            else:
                print(TAG+"msg_lst is empty!")

        tmg_true = float(my_msgs.read(CRS))

        #ribbon.hub_clear()
        if var_dir == "E":   # Correct for variation
            trk_mag = tmg_true - var_val  # NOTE !!! this is the opposite calculation than from magnetic +- variation to true heading
        elif var_dir == "W":
            trk_mag = tmg_true + var_val  # idem
        if trk_mag >= 360.0:
            trk_mag -= 360.0
        
        print(TAG+"track made good true: {}, var {} {}, track magnetcic: {}".format(tmg_true, var_val, var_dir, trk_mag), end='\n')
        tmg_true = round(tmg_true, 1)
        s_tmg_true = str(tmg_true)
        trk_mag = round(trk_mag, 1)
        s_trk_mag = str(trk_mag)
        if lDispMagOrTru:
            s = "TRACK " + s_trk_mag + " degs (M)"
        else:
            s = "TRACK " + s_tmg_true + " degs (T)"
        #if my_debug:
        print(TAG+s)
        print(TAG+s_telapsed+"{:5.2f} in mSecs".format(time_elapsed(loop_time, time.ticks_ms())), end="\n")

        if lDispMagOrTru:
            ribbon.set_heading_fm_sim(trk_mag)
        else:
            ribbon.set_heading_fm_sim(tmg_true)

        gr.clear()
        if startup == -1 or lTrackDirChgd:
            lTrackDirChgd = False # reset flag
            if lDispMagOrTru:
                scroll_text("TRK MAG", False)
            else:
                scroll_text("TRK TRUE", False)
            time.sleep(2)
            gr.clear
        gr.set_pen(gr.create_pen(int(BACKGROUND_COLOUR[0]), int(BACKGROUND_COLOUR[1]), int(BACKGROUND_COLOUR[2])))
        ribbon.ribbon_base()

        if biLdIsOn:
            led_toggle()

    except KeyboardInterrupt:
        return False
    return True

def disp_pos():
    TAG="disp_pos(): "
    lat = float(my_msgs.read(LAT))
    latdir = my_msgs.read(LATDIR)
    lon = float(my_msgs.read(LON))
    londir = my_msgs.read(LONDIR)
    s1 = "{:5.2f} {:s}".format(lat, latdir)
    s2 = "{:5.2f} {:s}".format(lon, londir)
    gr.clear()
    gr.set_pen(WHITE)
    print(TAG+"Pos= {s1}/{s2}")
    scroll_text(s1, False)
    print(TAG+s_telapsed+"{:5.2f} in mSecs".format(time_elapsed(loop_time, time.ticks_ms())), end="\n")
    time.sleep(2)
    gr.clear()
    print(TAG+s2)
    scroll_text(s2, False)
    time.sleep(2)
    return True

def disp_gs():
    TAG= "disp_gs(): "
    t_gs = "GS {:d} KT".format(int(ck_gs()))
    gr.clear()
    gr.set_pen(WHITE)
    #outline_text("Disp GS", 4, 2, cnt=0)
    print(TAG, t_gs)
    scroll_text(t_gs, False)
    print(TAG+s_telapsed+"{:5.2f} in mSecs".format(time_elapsed(loop_time, time.ticks_ms())), end="\n")
    time.sleep(3)
    return True

def disp_alt():
    TAG="disp_alt(): "
    t_alt = "A {:s} FT".format(my_msgs.read(ALT)) # ALT is an integer
    gr.clear()
    gr.set_pen(WHITE)
    #outline_text("Disp ALT", 4, 2, cnt=0)
    print(TAG+f"ALT= ", t_alt)
    scroll_text(t_alt, False)
    print(TAG+s_telapsed+"{:5.2f} in mSecs".format(time_elapsed(loop_time, time.ticks_ms())), end="\n")
    time.sleep(3)
    return True

"""
intro(lIntroShown, lSyncTime) -> void
        This function is called by main()
        Parameter: boolean: lIntroShown.  Flag to signal that the intro messages have been displayed
        Parameter: boolean: lSyncTime. Flag to call sync_time() or not
        Return: boolean: lIntroShown
"""
def intro(intro_shown, lSyncTime):
    if not intro_shown:
        intro_shown = True
        gr.clear()
        scroll_text("MSFS 2020", False)
        time.sleep(3)
        gr.clear()
        scroll_text("GPS RX", False)
        if lSyncTime:
            sync_time(False)  # get NTP time
        else:
            time.sleep(3)
        gr.clear()
        year, month, day, wd, hour, minute, second, _ = rtc.datetime()
        ymd = "{:04} {:02} {:02}".format(year, month, day)
        hms = "{:02} {:02} {:02}".format(hour, minute, second)

        # Year / Month / Day
        ribbon.draw_number(8, 6, ymd)
        time.sleep(3)
        
        # Hour / Minute / Second
        ribbon.draw_number(1, 1, hms) 
        time.sleep(4)
    return intro_shown

"""
main(void) -> void
        This is the start function
        Parameters: None
        Return: None
"""
def main(Slow):
    global my_debug, height, biLdIsOn, set_rgb, led, state, ribbon
    TAG = "main(): "
    lStart = True
    lUpdate = False
    intro_shown = False

    # ----------------------------------------------+
    # Settings for Galactic Unicorn
    #if lStart:
    #    #print("Starting GPRMC & GPGGA GPSout RX  53x11 Galactic Unicorn)
    # Lines for use with Heading Ribbon
    if my_debug:
        print("main(): we passed here")
    # Create ribbon object
    ribbon = HdgRibbon()
    
    time_last = time.ticks_ms()  #time.ticks_ms()

    led.toggle()
    biLdIsOn = False
    
    if not intro_shown:
        intro_shown = intro(intro_shown, True) 

    #sync_time(False)  # get NTP time
    
    if Slow:
        MESSAGE = "              MSFS 2020 GPRMC AND GPGGA GPS MESSAGES RX FOR PIMORONI\'S GALACTIC UNICORN                           "
        scroll_text(MESSAGE, True)
        time.sleep(3)
        gr.set_pen(gr.create_pen(0, 0, 0))
        gr.clear()
        gu.update(gr)
    
    stop = False
    
    if my_debug:
        print("main(): entering main loop")
    while True:
        try:
            # ----------------------------------------------+
            # Settings for Galactic Unicorn
            t = time.ticks_ms()
            
            gr.clear()
    
            # ----------FROM SCROLLING-TEXT SCRIPT FOR GALACTIC UNICORN ------------------------------------+
            time_now = time.ticks_ms()
            time_delta = time_now - time_last

            if lStart or lUpdate:
                lStart = False
                lUpdate = False
                gr.clear()

            time.sleep(0.001)
            time_last = time_now
            
            """
            if gu.is_pressed(gu.SWITCH_BRIGHTNESS_UP):
                   handle_lux_up(gu.SWITCH_BRIGHTNESS_UP)

            if gu.is_pressed(gu.SWITCH_BRIGHTNESS_DOWN):
                handle_lux_up(gu.SWITCH_BRIGHTNESS_DN)
            
            if use_sound:            
                if gu.is_pressed(gu.SWITCH_VOLUME_UP):
                    handle_vol_up(gu.SWITCH_VOLUME_UP)

                if gu.is_pressed(gu.SWITCH_VOLUME_DOWN):
                    handle_vol_dn(gu.SWITCH_VOLUME_DOWN)

            if gu.is_pressed(gu.SWITCH_A):
                handle_a(gu.SWITCH_A)
            
            if gu.is_pressed(gu.SWITCH_B):
                handle_b(gu.SWITCH_B)
                
            if gu.is_pressed(gu.SWITCH_C):
                handle_c(gu.SWITCH_C)
                
            if gu.is_pressed(gu.SWITCH_D):
                handle_d(gu.SWITCH_D)
                
            if gu.is_pressed(gu.SWITCH_SLEEP):
                handle_rst(gu.SWITCH_SLEEP)
            """
            if not loop():
                print(TAG+"loop() return with error")
                time.sleep(3)
        except KeyboardInterrupt:
           stop = True
           break
    if stop:
        print(TAG+"User interrupt. Exiting...")
        sys.exit()

# Call the main function
if __name__ == '__main__':
    main(False)

# ----- END-OF-SKETCH -----



