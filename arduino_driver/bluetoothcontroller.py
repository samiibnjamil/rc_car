import evdev
import serial
from threading import Thread
import time

def find_controller_by_name(controller_name):
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if controller_name in device.name:
            return device
    return None

# Define global variables to store button values
button_x = button_y = button_a = button_b = 0
joystickL_x = joystickL_y = joystickR_x = joystickR_y = 0
L2_Button = R2_Button = 0

# Define the Arduino serial port
arduino_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)

# Function to update global button values
def update_button_values(event):
    global button_x, button_y, button_a, button_b, joystickL_x, joystickL_y, joystickR_x, joystickR_y, L2_Button, R2_Button

    # Handle button events
    if event.type == evdev.ecodes.EV_KEY:
        key_code = event.code
        value = event.value

        if key_code == 307:
            button_x = value
        elif key_code == 308:
            button_y = value
        elif key_code == 304:
            button_a = value
        elif key_code == 305:
            button_b = value

    # Handle absolute axis events
    elif event.type == evdev.ecodes.EV_ABS:
        abs_code = event.code
        value = event.value

        if abs_code == evdev.ecodes.ABS_X:
            joystickL_x = value
        elif abs_code == evdev.ecodes.ABS_Y:
            joystickL_y = value
        elif abs_code == evdev.ecodes.ABS_RZ:
            joystickR_x = value
        elif abs_code == evdev.ecodes.ABS_Z:
            joystickR_y = value
        elif abs_code == evdev.ecodes.ABS_GAS:
            R2_Button = event.value
        elif abs_code == evdev.ecodes.ABS_BRAKE:
            L2_Button = event.value

# Function to read and print serial output from Arduino
def read_serial_from_arduino():
    while True:
        if arduino_serial.in_waiting > 0:
            arduino_data = arduino_serial.readline().decode().strip()
            print("Arduino:", arduino_data)



# Function to send data to Arduino via serial
def send_data_to_arduino():
    prev_data_string = ""
    while True:
        data_string = f"X,{button_x},Y,{button_y},A,{button_a},B,{button_b},JoystickL_X,{joystickL_x},JoystickL_Y,{joystickL_y},JoystickR_X,{joystickR_x},JoystickR_Y,{joystickR_y},L2,{L2_Button},R2,{R2_Button}"
        
        # Check if the data has changed
        if data_string != prev_data_string:
            # Print the data for debugging
            print(data_string)

            # Send data to Arduino
            arduino_serial.write(data_string.encode())

            # Update previous data for the next iteration
            prev_data_string = data_string
        
        # Sleep for a short duration to avoid excessive data transmission
        time.sleep(0.1)

# Function to print values to the terminal for debugging
def print_values_to_terminal():
    while True:
        print(f"Button X: {button_x}, Button Y: {button_y}, Button A: {button_a}, Button B: {button_b}")
        print(f"Joystick L X: {joystickL_x}, Joystick L Y: {joystickL_y}")
        print(f"Joystick R X: {joystickR_x}, Joystick R Y: {joystickR_y}")
        print(f"L2 Button: {L2_Button}, R2 Button: {R2_Button}")

        # Sleep for a short duration to avoid excessive printing
        time.sleep(0.5)

# Create threads for data sending and printing values
data_thread = Thread(target=send_data_to_arduino)
#print_thread = Thread(target=print_values_to_terminal)
read_serial_thread = Thread(target=read_serial_from_arduino)

# Start both threads
data_thread.start()
#print_thread.start()
read_serial_thread.start()

# Set up the event device for the StadiaSYS6-6f3b controller
controller_name3 = "StadiaSYS6-6f3b"
controller_name = "StadiaMXT5-e3ac"
gamepad = find_controller_by_name(controller_name)

if gamepad is not None:
    print(f"Found {controller_name} controller at {gamepad.path}")
    
    # Monitor events and update global variables
    for event in gamepad.read_loop():
        update_button_values(event)
else:
    print(f"Controller {controller_name} not found.")
