# Import necessary modules
from machine import Pin, PWM
# import RPi.GPIO as GPIO
import bluetooth
from peripheral import BLESimplePeripheral
import utime

# Create a Bluetooth Low Energy (BLE) object
ble = bluetooth.BLE()

# Create an instance of the BLESimplePeripheral class with the BLE object
sp = BLESimplePeripheral(ble)

# Create a Pin object for the onboard LED, configure it as an output
#led = Pin("LED", Pin.OUT)
led = Pin(0, Pin.OUT)
servo = PWM(Pin(16))
servo.freq(50)

degrees = 0

def move_servo(servo, angle):
    duty = int(((angle / 180) * 2000) + 500)
    servo.duty_ns(duty * 1000)  # duty_ns takes duty cycle in nanoseconds
    utime.sleep_ms(500)  # Wait for servo to reach the position


#Initialize the LED state to 0 (off)
led_state = 0
led.value(0)
(Pin("LED", Pin.OUT)).off()

move_servo(servo, 0)
utime.sleep(1)

moved = False


dimmedLEDs = [11, 12, 13]

dimmedLEDPWMs = [PWM(Pin(pin), freq=1000) for pin in dimmedLEDs]

def set_brightness(led_index, brightness):
    # Map the input from 0-255 to 0-1023 (PWM duty cycle)
    dimmedLEDPWMs[led_index].duty_u16(int(brightness * 4))

# GPIO.setmode(GPIO.BCM)
# 
# for pin in dimmedLEDs:
#     GPIO.setup(pin, GPIO.OUT)
#     
#     global pwms
#     pwms = [GPIO.PWM(pin, 100) for pin in dimmedLEDs]
#     
#     for pwm in pwms:
#         pwm.start(0)



def clamp(n_min, value, n_max):
    if value < n_min:
        return n_min
    if value > n_max:
        return n_max
    else:
        return value

# Define a callback function to handle received data
def on_rx(data):
    print("Data received: ", data)  # Print the received data
    global led_state  # Access the global variable led_state
    
    print(data)
    print(data.decode("utf-8").strip().split(","))
    
    dimData = data.decode("utf-8").strip().split(",")
    
    for i, dimmedLED in enumerate(dimData):
        set_brightness(i, clamp(0, int(dimmedLED), 255))

#     
#     if data == b'toggle\r\n':  # Check if the received data is "toggle"
#         led.value(not led_state)  # Toggle the LED state (on/off)
#         led_state = 1 - led_state  # Update the LED state
#         
#         moved = False
#         
#         if not moved:
#             if led_state:
#                 print(90)
#                 move_servo(servo, 90)
#                 
#             else:
#                 print(0)
#                 move_servo(servo, 0)
#             moved = True
        

# Start an infinite loop
while True:
    if sp.is_connected():  # Check if a BLE connection is established
        
        sp.on_write(on_rx)  # Set the callback function for data reception
        
