# ROS Touchscreen Joystick Controller

## Hardware Components
- Raspberry Pi 5 (main computing unit)
- Teensy Microcontroller
- 7-inch Raspberry Pi Touchscreen Module
- Two Arduino Joystick Modules
- Emergency Stop Button
- Three Additional Control Buttons

## System Architecture
```
[Joysticks & Buttons] -> [Teensy] --Serial--> [Raspberry Pi] --ROS2--> [Robot Systems]
```

## Setup and Installation

### Hardware Connections
1. Connect joysticks and buttons to Teensy
2. Connect Teensy to Raspberry Pi via USB/Serial
3. Mount in enclosed case

### Teensy Firmware
```cpp
// teensy_input_controller.ino
void setup() {
  Serial.begin(115200);  // Match baudrate in ROS2 node
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(EMERGENCY_STOP, INPUT_PULLUP);
}

void loop() {
  int leftX = analogRead(LEFT_JOYSTICK_X);
  int leftY = analogRead(LEFT_JOYSTICK_Y);
  bool emergencyStop = digitalRead(EMERGENCY_STOP);
  
  // Construct serial message
  String message = String(leftX) + "," + 
                   String(leftY) + "," + 
                   String(emergencyStop);
  
  Serial.println(message);
  delay(50);  // Polling interval
}
```

### Raspberry Pi ROS2 Node
```python
# ros2_serial_node.py
import rclpy
import serial

class JoystickSerialNode:
    def __init__(self):
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200)
        self.pub_joystick = self.create_publisher(Joy, 'dalek_teleop')
    
    def read_serial_data(self):
        data = self.serial_port.readline().decode().strip()
        x, y, emergency_stop = map(int, data.split(','))
        
        joy_msg = Joy()
        joy_msg.axes = [x/1023.0, y/1023.0]  # Normalize
        joy_msg.buttons = [emergency_stop]
        
        self.pub_joystick.publish(joy_msg)

def main():
    rclpy.init()
    node = JoystickSerialNode()
    rclpy.spin(node)
```

### Dependencies
```bash
pip install pyserial
sudo apt install ros-humble-joy
```

## Additional Configuration Notes
- Calibrate joystick ranges in Teensy firmware
- Ensure consistent serial communication parameters
- Map input ranges to ROS Joy message standards

## Troubleshooting
- Verify serial port connection
- Check Teensy firmware upload
- Validate ROS2 topic publishing
