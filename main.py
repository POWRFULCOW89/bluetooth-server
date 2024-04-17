# Import necessary modules
from machine import Pin, PWM
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

# Define a callback function to handle received data
def on_rx(data):
    print("Data received: ", data)  # Print the received data
    global led_state  # Access the global variable led_state
    if data == b'toggle\r\n':  # Check if the received data is "toggle"
        led.value(not led_state)  # Toggle the LED state (on/off)
        led_state = 1 - led_state  # Update the LED state
        
        moved = False
        
        if not moved:
            if led_state:
                print(90)
                move_servo(servo, 90)
                
            else:
                print(0)
                move_servo(servo, 0)
            moved = True
        

# Start an infinite loop
while True:
    if sp.is_connected():  # Check if a BLE connection is established
        
        sp.on_write(on_rx)  # Set the callback function for data reception
        
        
