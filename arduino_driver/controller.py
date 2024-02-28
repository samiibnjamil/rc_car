from evdev import InputDevice, list_devices, ecodes

def get_xbox_controller_path():
    # Find the input device path for the Xbox controller
    xbox_vendor_id = 0x045e  # Replace with your Xbox controller's vendor ID
    xbox_product_id = 0x028e  # Replace with your Xbox controller's product ID

    devices = [InputDevice(device) for device in list_devices()]

    for device in devices:
        if device.info.vendor == xbox_vendor_id and device.info.product == xbox_product_id:
            return device.path

    return None

def get_xbox_controller_events(device_path):
    device = InputDevice(device_path)

    joystick1_x = joystick1_y = joystick2_x = joystick2_y = 0
    LT_Button = RT_Button = 0  # Initialize LT_Button and RT_Button

    for event in device.read_loop():
        if event.type == ecodes.EV_KEY:
            print(f"Button,{event.code},{event.value}")
        elif event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X:
                joystick1_x = event.value
            elif event.code == ecodes.ABS_Y:
                joystick1_y = event.value
            elif event.code == ecodes.ABS_RX:
                joystick2_x = event.value
            elif event.code == ecodes.ABS_RY:
                joystick2_y = event.value
            elif event.code == ecodes.ABS_Z:
                LT_Button = event.value
            elif event.code == ecodes.ABS_RZ:
                RT_Button = event.value

            print(f"LT_Button,{LT_Button},RT_Button,{RT_Button},Joystick1_X,{joystick1_x},Joystick1_Y,{joystick1_y},Joystick2_X,{joystick2_x},Joystick2_Y,{joystick2_y}")

if __name__ == "__main__":
    xbox_controller_path = get_xbox_controller_path()

    if xbox_controller_path:
        print("Xbox controller found")
        get_xbox_controller_events(xbox_controller_path)
    else:
        print("Xbox controller not found.")
