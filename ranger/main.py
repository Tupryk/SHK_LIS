import can
import time

# Define CAN ID for communication
SEND_ID = 0x211  # ID for sending commands
RECEIVE_ID = 0x211  # ID for receiving feedback
CAN_INTERFACE = 'can0'  # Replace with your CAN interface name

# Initialize CAN bus
bus = can.interface.Bus(channel=CAN_INTERFACE, interface='slcan', bitrate=500000)

# Example function to send a command
def send_command(control_mode, linear_speed, steering_angle):
    """
    Send a command to the Agilex Ranger.
    :param control_mode: Control mode (0x00: Standby, 0x01: Command control, 0x03: Remote control)
    :param linear_speed: Linear speed in m/s (scaled to fit the protocol)
    :param steering_angle: Steering angle in degrees (scaled to fit the protocol)
    """
    # Scale linear speed and steering angle
    linear_speed_scaled = int(linear_speed * 1000)  # Example scaling
    steering_angle_scaled = int(steering_angle * 100)  # Example scaling

    # Create data payload (example format)
    data = [
        0x00,  # Byte 0: Current vehicle status (dummy, handled by Ranger)
        control_mode,  # Byte 1: Control mode
        (linear_speed_scaled >> 8) & 0xFF,  # Byte 2: High byte of linear speed
        linear_speed_scaled & 0xFF,  # Byte 3: Low byte of linear speed
        (steering_angle_scaled >> 8) & 0xFF,  # Byte 4: High byte of steering angle
        steering_angle_scaled & 0xFF,  # Byte 5: Low byte of steering angle
        0x00,  # Byte 6: Reserved
        0x00   # Byte 7: Reserved
    ]

    # Send CAN message
    msg = can.Message(arbitration_id=SEND_ID, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"Command sent: {data}")
    except can.CanError as e:
        print(f"Error sending command: {e}")

# Example function to receive feedback
def receive_feedback():
    """
    Receive feedback from the Agilex Ranger.
    """
    try:
        message = bus.recv(timeout=1.0)  # Wait for 1 second
        if message and message.arbitration_id == RECEIVE_ID:
            data = message.data
            # Parse the feedback data
            vehicle_status = data[0]
            control_mode = data[1]
            battery_voltage = (data[2] << 8 | data[3]) / 10.0
            error_code = (data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7])

            print(f"Vehicle Status: {vehicle_status}")
            print(f"Control Mode: {control_mode}")
            print(f"Battery Voltage: {battery_voltage} V")
            print(f"Error Code: {error_code:#010x}")
        else:
            print("No feedback received or unknown message ID.")
    except can.CanError as e:
        print(f"Error receiving feedback: {e}")

# Main function
if __name__ == "__main__":
    try:
        # Example usage
        send_command(control_mode=0x01, linear_speed=1.0, steering_angle=15.0)
        time.sleep(0.1)
        receive_feedback()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        bus.shutdown()
