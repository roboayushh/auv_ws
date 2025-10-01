import rclpy
from rclpy.node import Node
from joystick_msgs.msg import JoystickData
import serial
import numpy as np

class ROVPWMController(Node):
    def __init__(self):
        super().__init__('rov_pwm_controller')

        # --- Parameters ---
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        port = self.get_parameter('arduino_port').get_parameter_value().string_value
        
        # --- Arduino Connection ---
        try:
            self.arduino = serial.Serial(port, baudrate=115200, timeout=0.1)
            self.get_logger().info(f"Successfully connected to Arduino on port {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino on port {port}: {e}")
            # Exit or handle the error appropriately
            rclpy.shutdown()
            return

        # --- Thruster Mixing Matrix (6x6) ---
        # This matrix defines how joystick inputs map to the 6 thrusters
        self.mixing_matrix = np.array([
            [1,  0,  1,  1, 0,  0],  # T1 (surge, sway, heave, roll, pitch, yaw)
            [1,  0, -1, -1, 0,  0],  # T2
            [1,  0, -1,  1, 0,  0],  # T3
            [1,  0,  1, -1, 0,  0],  # T4
            [0,  1,  0,  0, 0,  1],  # T5
            [0,  1,  0,  0, 0, -1],  # T6
        ])

        # --- Subscriber ---
        # Subscribes to the joystick data topic. When a message is received,
        # the joystick_data_callback function is executed.
        self.create_subscription(
            JoystickData,
            'joystick_data',
            self.joystick_data_callback,
            10
        )

        self.get_logger().info("ROV PWM Controller is running and waiting for joystick data...")

    def joystick_data_callback(self, msg):
        """
        This function is called every time a new message is received on the 'joystick_data' topic.
        """
        # 1. Create a numpy array from the incoming joystick message
        thrust_input = np.array([
            msg.x,      # Corresponds to surge
            msg.y,      # Corresponds to sway
            msg.z,      # Corresponds to heave
            msg.yaw,    # Corresponds to yaw
            msg.pitch,  # Corresponds to pitch
            msg.roll    # Corresponds to roll
        ])

        # 2. Log the received data for debugging
        self.get_logger().info(f"Received Joystick Data: {thrust_input}")

        # 3. Apply the mixing matrix to calculate individual thruster outputs
        # The '@' symbol is matrix multiplication
        thrust_output = self.mixing_matrix @ thrust_input
        
        # 4. Clip the values to ensure they are within the [-1.0, 1.0] range
        thrust_output = np.clip(thrust_output, -1.0, 1.0)
        
        # 5. Convert the [-1.0, 1.0] values to a PWM signal range [1100, 1900]
        # 1500 is neutral, 1100 is full reverse, 1900 is full forward.
        thrust_pwm = [int(1500 + 400 * t) for t in thrust_output]

        # 6. Format the PWM values into a comma-separated string with a newline
        packet = ','.join(str(p) for p in thrust_pwm) + '\n'
        
        # 7. Send the formatted string to the Arduino
        try:
            self.arduino.write(packet.encode('utf-8'))
            self.get_logger().info(f"Sent PWM to Arduino: {packet.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ROVPWMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the serial connection is closed when the node is shut down
        if node.arduino and node.arduino.is_open:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()