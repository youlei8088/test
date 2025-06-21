import serial
import serial.tools.list_ports

class SerialHandler:
    """
    Handles serial communication with the motor controller firmware.
    This includes discovering available serial ports, connecting/disconnecting,
    and sending specific commands for motor control (enable, disable, set zero).
    """
    def __init__(self, logger=None):
        """
        Initializes the SerialHandler.

        Args:
            logger (GUILogger, optional): An instance of GUILogger for logging messages.
                                          If None, messages are printed to stdout.
        """
        self.ser = None  # Will hold the serial.Serial object when connected
        self.logger = logger

    def _log(self, message):
        """Internal helper to log messages via the provided logger or print."""
        if self.logger:
            self.logger.log(f"[SerialHandler] {message}")
        else:
            print(f"[SerialHandler] {message}")

    def get_available_ports(self):
        """
        Lists available serial ports, prioritizing Silicon Labs CP210x devices.

        Returns:
            list[str]: A list of port names (e.g., ['COM4', 'COM5']).
                       CP210x devices are listed first if found.
        """
        ports = serial.tools.list_ports.comports()
        available_ports = []
        cp210x_ports = []

        for port_info in ports:
            port_name = port_info.device
            # Add description and VID:PID for better identification if needed for logging
            # desc = port_info.description
            # vid = port_info.vid
            # pid = port_info.pid
            # self._log(f"Found port: {port_name}, Desc: {desc}, VID: {vid:04X}, PID: {pid:04X}")

            available_ports.append(port_name)
            # Check for Silicon Labs CP210x (common VID:PID is 10C4:EA60)
            # The string "CP210x" in description is often a reliable indicator.
            if "CP210x" in port_info.description or \
               (port_info.vid == 0x10C4 and port_info.pid == 0xEA60):
                cp210x_ports.append(port_name)

        # Prioritize CP210x ports by putting them at the beginning of the list
        if cp210x_ports:
            # Remove cp210x_ports from available_ports to avoid duplicates, then prepend
            final_port_list = list(cp210x_ports) # Start with CP210x ports
            for p in available_ports:
                if p not in final_port_list:
                    final_port_list.append(p)
            return final_port_list
        return available_ports

    def connect(self, port_name, baudrate=115200, timeout=1):
        """
        Connects to the specified serial port.

        Args:
            port_name (str): The name of the serial port (e.g., 'COM4' or '/dev/ttyUSB0').
            baudrate (int, optional): The baud rate for communication. Defaults to 115200.
            timeout (int, optional): Read timeout in seconds. Defaults to 1.

        Returns:
            bool: True if connection was successful, False otherwise.
        """
        if self.ser and self.ser.is_open:
            if self.ser.port == port_name:
                self._log(f"Already connected to {self.ser.port}.")
                return True
            else: # Connected to a different port, disconnect first
                self.disconnect()

        try:
            self.ser = serial.Serial(port_name, baudrate, timeout=timeout)
            if self.ser.is_open: # Double check
                self._log(f"Successfully connected to {port_name} at {baudrate} baud.")
                return True
            else: # Should not happen if serial.Serial() doesn't raise exception
                self._log(f"Failed to open {port_name} (is_open is false after connect).")
                self.ser = None
                return False
        except serial.SerialException as e:
            self._log(f"Error connecting to {port_name}: {e}")
            self.ser = None
            return False

    def disconnect(self):
        """
        Disconnects from the currently connected serial port.
        """
        if self.ser and self.ser.is_open:
            port_name = self.ser.port
            try:
                self.ser.close()
                self._log(f"Disconnected from {port_name}.")
            except Exception as e: # Catch any potential errors during close
                self._log(f"Error while closing port {port_name}: {e}")
            finally:
                self.ser = None
        else:
            self._log("Not connected to any serial port, or port already closed.")
            self.ser = None # Ensure it's None

    def send_command(self, command_char):
        """
        Sends a single character command to the connected serial port.
        The command character is encoded to ASCII before sending.

        Args:
            command_char (str): The single character command to send (e.g., 'm', 'z').

        Returns:
            bool: True if the command was sent successfully, False otherwise.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(command_char.encode('ascii'))
                self._log(f"Sent command: '{command_char}'")
                # Firmware might send an acknowledgment. Add read logic here if needed.
                # Example: response = self.ser.readline().decode('ascii').strip()
                # if response: self._log(f"Received response: '{response}'")
                return True
            except serial.SerialTimeoutException: # This is for write timeout if configured
                self._log(f"Timeout sending command '{command_char}'.")
                return False
            except Exception as e: # Catch other potential pyserial errors
                self._log(f"Error sending command '{command_char}': {e}")
                return False
        else:
            self._log("Serial port not connected. Cannot send command.")
            return False

    def enable_motor(self):
        """Sends the 'm' command to enable the motor."""
        return self.send_command('m')

    def disable_motor(self):
        """Sends the ESC command (ASCII character 27) to disable the motor."""
        esc_char = chr(27) # Standard ESC character
        return self.send_command(esc_char)

    def set_zero_point(self):
        """Sends the 'z' command to set the current motor position as the zero point."""
        return self.send_command('z')

if __name__ == '__main__':
    # This section provides an example of how to use SerialHandler directly for testing.
    # It requires a physical or virtual serial port to connect to.

    # A simple logger class for standalone testing
    class SimpleTestLogger:
        def log(self, message):
            print(message) # Print directly for console testing

    logger = SimpleTestLogger()
    handler = SerialHandler(logger=logger)

    print("--- Serial Handler Test ---")
    ports = handler.get_available_ports()
    print("\nAvailable serial ports:")
    if ports:
        for i, p in enumerate(ports):
            print(f"  {i}: {p}")
    else:
        print("  No serial ports found.")
        print("\nSerial Handler test finished (no ports to test connection).")
        exit()

    # Example: Try to connect to the first available port
    # In a real test, you might want to specify a known test port.
    target_port_index = 0
    if len(ports) > target_port_index:
        selected_port = ports[target_port_index]
        print(f"\nAttempting to connect to: {selected_port}")

        if handler.connect(selected_port):
            print(f"Successfully connected to {selected_port}.")

            print("\nTesting commands (these will be sent to the connected port):")
            # Note: Sending these commands to a random device might have unintended consequences.
            # Only proceed if you know what device is on `selected_port`.

            # print("  Sending 'Enable Motor' ('m')...")
            # handler.enable_motor()
            # # time.sleep(0.1) # Give device time to react if needed

            # print("  Sending 'Set Zero Point' ('z')...")
            # handler.set_zero_point()
            # # time.sleep(0.1)

            # print("  Sending 'Disable Motor' (ESC)...")
            # handler.disable_motor()

            print("\n(Command sending examples commented out for safety.)")
            print("To test command sending, uncomment lines in the __main__ block.")

            print("\nDisconnecting...")
            handler.disconnect()
            print("Disconnected.")
        else:
            print(f"Could not connect to {selected_port}.")
    else:
        print(f"Port index {target_port_index} is out of range.")

    print("\nSerial Handler test finished.")
