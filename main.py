import time
from mqtt import MQTTClient
from machine import Pin, ADC
import neopixel
import keys
import wifiConnection

# Settings
VOLTAGE_INTERVAL = 20000  # milliseconds, sending every 20 seconds to avoid throttling
last_voltage_sent_ticks = 0  # milliseconds
VOLTAGE_THRESHOLD = 3.175  # Voltage threshold to turn on the LED strip
MOTION_DEBOUNCE_TIME = 5  # seconds to keep the LED on after motion is detected

# Pin setups
pixSize = 60
pix = neopixel.NeoPixel(Pin(18), pixSize)
pir = Pin(21, Pin.IN, Pin.PULL_UP)
ldr = ADC(Pin(27))

# Variables to store RGB values
red_value = 155
green_value = 155
blue_value = 50
led_on = False
manual_control = False  # Flag to indicate if the LED is controlled manually via the dashboard
last_motion_time = 0  # Time when motion was last detected

# Function to set RGB values to the LED strip
def set_rgb(red, green, blue):
    for i in range(pixSize):
        pix[i] = (red, green, blue)
    pix.write()

# Function to turn on LED strip
def turn_on_led_strip():
    global led_on
    led_on = True
    set_rgb(red_value, green_value, blue_value)

# Function to turn off LED strip
def turn_off_led_strip():
    global led_on
    led_on = False
    set_rgb(0, 0, 0)  # Turn off by setting to black

# Callback Function to respond to messages from Adafruit IO
def sub_cb(topic, msg):
    global red_value, green_value, blue_value
    global manual_control

    print((topic, msg))  # Outputs the message that was received. Debugging use.

    if topic == bytes(keys.AIO_RED_FEED, 'utf-8'):
        red_value = int(msg)
        print(f"Red value set to: {red_value}")
    elif topic == bytes(keys.AIO_GREEN_FEED, 'utf-8'):
        green_value = int(msg)
        print(f"Green value set to: {green_value}")
    elif topic == bytes(keys.AIO_BLUE_FEED, 'utf-8'):
        blue_value = int(msg)
        print(f"Blue value set to: {blue_value}")
    elif topic == bytes(keys.AIO_MANUAL_CONTROL_FEED, 'utf-8'):
        manual_control = (msg == b'ON')
        print(f"Manual control set to: {manual_control}")

    # If in manual control, set the RGB values
    if manual_control:
        if red_value == 0 and green_value == 0 and blue_value == 0:
            turn_off_led_strip()
        else:
            turn_on_led_strip()
            set_rgb(red_value, green_value, blue_value)
            print(f"Set RGB to: ({red_value}, {green_value}, {blue_value})")

# Function to read ADC value and convert to voltage
def read_voltage(ldr):
    digital_value = ldr.read_u16()
    print("ADC value=", digital_value)
    voltage = 3.3 * (digital_value / 65535)
    print("Voltage: {}V ".format(voltage))
    return voltage

# Function to publish voltage to Adafruit IO MQTT server at fixed interval
def send_voltage():
    global last_voltage_sent_ticks

    if ((time.ticks_ms() - last_voltage_sent_ticks) < VOLTAGE_INTERVAL):
        return  # Too soon since last one sent.

    voltage = read_voltage(ldr)
    print("Publishing: {0:.2f}V to {1} ... ".format(voltage, keys.AIO_VOLTAGE_FEED), end='')
    try:
        client.publish(topic=keys.AIO_VOLTAGE_FEED, msg=str(voltage))
        print("DONE")
    except Exception as e:
        print("FAILED")
    finally:
        last_voltage_sent_ticks = time.ticks_ms()

# Function to publish motion data to Adafruit IO MQTT server
def send_motion(motion_detected):
    print(f"Publishing motion data: {motion_detected} to {keys.AIO_MOTION_FEED} ... ", end='')
    try:
        client.publish(topic=keys.AIO_MOTION_FEED, msg=str(motion_detected))
        print("DONE")
    except Exception as e:
        print("FAILED")

# Function to check for motion and voltage conditions
def check_motion_and_voltage():
    global last_motion_time
    global manual_control
    global led_on
    voltage = read_voltage(ldr)
    motion_detected = pir.value() == 1  # High signal when motion is detected

    print(f"Motion detected: {motion_detected}, Voltage: {voltage}")

    if not manual_control:
        if motion_detected and voltage <= VOLTAGE_THRESHOLD:
            last_motion_time = time.time()
            if not led_on:
                print("Motion detected and voltage <= 3.17V, turning on LED strip")
                turn_on_led_strip()
                send_motion(1)  # Send motion detected
        elif time.time() - last_motion_time > MOTION_DEBOUNCE_TIME:
            if led_on:
                print("Conditions not met: turning off LED strip")
                turn_off_led_strip()
                send_motion(0)  # Send no motion detected

# Try WiFi Connection
try:
    ip = wifiConnection.connect()
    print(f"Connected to WiFi, IP address: {ip}")
except Exception as e:
    print(f"Failed to connect to WiFi: {e}")
    raise

# MQTT to connect to Adafruit IO
try:
    client = MQTTClient(keys.AIO_CLIENT_ID, keys.AIO_SERVER, keys.AIO_PORT, keys.AIO_USER, keys.AIO_KEY)
    client.set_callback(sub_cb)
    client.connect()
    print("Connected to MQTT server")
except Exception as e:
    print(f"Failed to connect to MQTT server: {e}")
    raise

# Subscribe to RGB and manual control feeds
try:
    client.subscribe(bytes(keys.AIO_RED_FEED, 'utf-8'))
    client.subscribe(bytes(keys.AIO_GREEN_FEED, 'utf-8'))
    client.subscribe(bytes(keys.AIO_BLUE_FEED, 'utf-8'))
    client.subscribe(bytes(keys.AIO_MANUAL_CONTROL_FEED, 'utf-8'))
    print("Subscribed to RGB and manual control feeds")
except Exception as e:
    print(f"Failed to subscribe to feeds: {e}")
    raise

print("Connected to %s, subscribed to %s, %s, %s, %s topics" % (keys.AIO_SERVER, keys.AIO_RED_FEED, keys.AIO_GREEN_FEED, keys.AIO_BLUE_FEED, keys.AIO_MANUAL_CONTROL_FEED))

try:
    while True:
        client.check_msg()  # Action a message if one is received. Non-blocking.
        send_voltage()      # Send voltage to Adafruit IO if it's time.
        check_motion_and_voltage()  # Check motion and voltage conditions
        time.sleep(1)       # Add a delay to manage the frequency of the loop
finally:
    client.disconnect()
    client = None
    wifiConnection.disconnect()
    print("Disconnected from Adafruit IO.")
