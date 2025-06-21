import ctypes
import os
import time # For sleep in test __main__
import platform # For checking OS type if needed for DLL paths
import sys # For PyInstaller _MEIPASS check
from test.can_communication.message_packer import MessagePacker

# --- Constants for ControlCAN.dll ---
# These values are typically found in the ControlCAN.h header file provided with the DLL.
# They might need adjustment if a different version of the DLL or hardware is used.

# Device Types (from ZLG documentation for CANalyst-II)
VCI_USBCAN1 = 3         # Older USBCAN device
VCI_USBCAN2 = 4         # Common USBCAN-II device (dual channel)
VCI_USBCAN_E_U = 20     # USBCAN-E-U
VCI_USBCAN_2E_U = 21    # USBCAN-2E-U (dual channel)

# Status Codes (common values, 1 for OK, 0 for Error)
STATUS_OK = 1
STATUS_ERR = 0 # Often, functions return 0 on error, or specific error codes (like 0xFFFFFFFF)

# CAN Message Object Structure (VCI_CAN_OBJ)
# This structure defines how a CAN frame is represented when passing to/from the DLL.
class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint),          # CAN ID (standard or extended)
        ("TimeStamp", ctypes.c_uint),   # Timestamp of the frame (hardware/driver provided)
        ("TimeFlag", ctypes.c_ubyte),   # Whether TimeStamp is valid
        ("SendType", ctypes.c_ubyte),   # Send type (0:normal, 1:single, 2:self-rx, 3:single-self-rx)
        ("RemoteFlag", ctypes.c_ubyte), # 0: Data Frame, 1: Remote Frame
        ("ExternFlag", ctypes.c_ubyte), # 0: Standard Frame (11-bit ID), 1: Extended Frame (29-bit ID)
        ("DataLen", ctypes.c_ubyte),    # Length of data (0-8 bytes)
        ("Data", ctypes.c_ubyte * 8),   # CAN data payload (max 8 bytes)
        ("Reserved", ctypes.c_ubyte * 3) # Reserved bytes
    ]

# Initialization Configuration Structure (VCI_INIT_CONFIG)
# Used to configure a CAN channel (baud rate, filter, mode).
class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_ulong),    # Acceptance Code for hardware filtering
        ("AccMask", ctypes.c_ulong),    # Acceptance Mask for hardware filtering
        ("Reserved", ctypes.c_ulong),   # Reserved
        ("Filter", ctypes.c_ubyte),     # Filter type (0/1: dual filter, 2: single filter)
        ("Timing0", ctypes.c_ubyte),    # Baud rate timing register 0 (e.g., 0x00 for 1Mbps)
        ("Timing1", ctypes.c_ubyte),    # Baud rate timing register 1 (e.g., 0x14 for 1Mbps)
        ("Mode", ctypes.c_ubyte)        # Mode (0: normal, 1: loopback, 2: listen-only)
    ]


class CANHandler:
    """
    Manages communication with a CAN device (e.g., ZLG CANalyst-II) using ControlCAN.dll.
    Provides methods to open/close the device, initialize CAN channels, and send/receive CAN messages.
    It uses the MessagePacker class to format outgoing commands and parse incoming replies
    according to the motor firmware's specifications.
    """
    def __init__(self, dll_path="ControlCAN.dll", device_type=VCI_USBCAN2,
                 device_idx=0, can_idx=0, logger=None):
        """
        Initializes the CANHandler.

        Args:
            dll_path (str, optional): Path to the ControlCAN.dll file. Defaults to "ControlCAN.dll".
            device_type (int, optional): Type of the CAN device (e.g., VCI_USBCAN2). Defaults to VCI_USBCAN2.
            device_idx (int, optional): Index of the CAN device (if multiple are connected). Defaults to 0.
            can_idx (int, optional): Index of the CAN channel on the device (0 or 1 for dual-channel). Defaults to 0.
            logger (GUILogger, optional): Logger instance for messages. Defaults to None (prints to console).
        """
        self.dll_path = dll_path
        self.device_type = device_type
        self.device_idx = device_idx
        self.can_idx = can_idx

        self.can_lib = None     # Holds the loaded ctypes DLL object
        self.is_open = False    # Tracks if the CAN device is currently open and initialized
        self.logger = logger
        self.packer = MessagePacker(logger=logger) # For packing/unpacking CAN message payloads

        self._load_library() # Attempt to load the DLL upon instantiation

    def _log(self, message):
        """Internal helper to log messages, prepending class name."""
        if self.logger:
            self.logger.log(f"[CANHandler] {message}")
        else:
            print(f"[CANHandler] {message}")

    def _load_library(self):
        """
        Loads the ControlCAN.dll library using ctypes.
        It tries to find the DLL in common locations relative to the script or application.
        """
        try:
            dll_load_path = ""
            # Determine if running in a PyInstaller bundle
            if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
                # Running in a bundle, DLL should be in the _MEIPASS directory
                bundle_dir = sys._MEIPASS
                dll_load_path = os.path.join(bundle_dir, self.dll_path)
            else:
                # Running as a normal script, try various relative paths
                script_dir = os.path.dirname(os.path.abspath(__file__)) # dir of can_handler.py
                paths_to_try = [
                    self.dll_path,                                      # CWD
                    os.path.join(os.getcwd(), "test", self.dll_path),   # CWD/test/ControlCAN.dll (if run from project root)
                    os.path.join(script_dir, "..", self.dll_path),      # ../ControlCAN.dll (e.g., test/ControlCAN.dll)
                    os.path.join(script_dir, self.dll_path)             # ./ControlCAN.dll (alongside can_handler.py)
                ]

                # Note: For Windows, one might also check System32, but bundling or local placement is preferred.
                # if platform.system() == "Windows":
                #     paths_to_try.append(os.path.join(os.environ.get("SystemRoot", "C:\\Windows"), "System32", self.dll_path))

                found_path = None
                for p_try in paths_to_try:
                    if os.path.exists(p_try):
                        found_path = os.path.abspath(p_try) # Use absolute path for loading
                        break

                if not found_path:
                    self._log(f"Error: {self.dll_path} not found in expected locations: {paths_to_try}")
                    self.can_lib = None
                    return
                dll_load_path = found_path

            self._log(f"Attempting to load CAN library from: {dll_load_path}")
            self.can_lib = ctypes.cdll.LoadLibrary(dll_load_path)
            self._setup_api_prototypes() # Define argtypes and restypes for DLL functions
            self._log(f"{self.dll_path} loaded successfully from {dll_load_path}.")
        except OSError as e:
            self._log(f"OSError loading {self.dll_path}: {e}. "
                      f"Ensure the DLL is in the correct path and all its dependencies are met "
                      f"(e.g., correct MSVC++ Redistributable for its architecture).")
            self.can_lib = None
        except Exception as e: # Catch any other unexpected errors during loading
            self._log(f"An unexpected error occurred while loading {self.dll_path}: {e}")
            self.can_lib = None

    def _setup_api_prototypes(self):
        """
        Defines the argument types (argtypes) and return types (restype) for the ControlCAN API functions.
        This is crucial for ctypes to correctly call the DLL functions.
        These definitions should match the function signatures in ControlCAN.h.
        """
        if not self.can_lib: return

        # VCI_OpenDevice(DWORD DeviceType, DWORD DeviceInd, DWORD Reserved)
        self.can_lib.VCI_OpenDevice.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.can_lib.VCI_OpenDevice.restype = ctypes.c_ulong # Typically returns STATUS_OK or STATUS_ERR

        # VCI_CloseDevice(DWORD DeviceType, DWORD DeviceInd)
        self.can_lib.VCI_CloseDevice.argtypes = [ctypes.c_uint, ctypes.c_uint]
        self.can_lib.VCI_CloseDevice.restype = ctypes.c_ulong

        # VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig)
        self.can_lib.VCI_InitCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(VCI_INIT_CONFIG)]
        self.can_lib.VCI_InitCAN.restype = ctypes.c_ulong

        # VCI_StartCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd)
        self.can_lib.VCI_StartCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.can_lib.VCI_StartCAN.restype = ctypes.c_ulong

        # VCI_ResetCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd) - Optional, for resetting channel
        self.can_lib.VCI_ResetCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.can_lib.VCI_ResetCAN.restype = ctypes.c_ulong

        # VCI_StopCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd) - ZLG specific, good for cleanup
        self.can_lib.VCI_StopCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.can_lib.VCI_StopCAN.restype = ctypes.c_ulong

        # VCI_Transmit(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pSend, ULONG Len)
        self.can_lib.VCI_Transmit.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(VCI_CAN_OBJ), ctypes.c_uint]
        self.can_lib.VCI_Transmit.restype = ctypes.c_ulong # Returns number of frames sent, or error status

        # VCI_Receive(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pReceive, ULONG Len, INT WaitTime)
        self.can_lib.VCI_Receive.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(VCI_CAN_OBJ), ctypes.c_uint, ctypes.c_int]
        self.can_lib.VCI_Receive.restype = ctypes.c_ulong # Returns number of frames read, or error status

        # VCI_GetReceiveNum(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd)
        self.can_lib.VCI_GetReceiveNum.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.can_lib.VCI_GetReceiveNum.restype = ctypes.c_ulong # Returns number of frames in buffer

        # VCI_ClearBuffer(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd)
        self.can_lib.VCI_ClearBuffer.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.can_lib.VCI_ClearBuffer.restype = ctypes.c_ulong


    def open(self, acc_code=0x00000000, acc_mask=0xFFFFFFFF,
             timing0=0x00, timing1=0x14, mode=0): # Default to 1Mbps
        """
        Opens the CAN device, initializes a CAN channel, and starts it.
        Default baud rate is 1Mbps (Timing0=0x00, Timing1=0x14 for ZLG).
        Other common baud rates:
          - 500Kbps: Timing0=0x00, Timing1=0x1C
          - 250Kbps: Timing0=0x01, Timing1=0x1C
          - 125Kbps: Timing0=0x03, Timing1=0x1C

        Args:
            acc_code (int, optional): Acceptance code for message filtering. Defaults to 0 (receive all).
            acc_mask (int, optional): Acceptance mask for message filtering. Defaults to 0xFFFFFFFF (relevant for all bits).
            timing0 (int, optional): Baud rate timing register 0. Defaults to 0x00.
            timing1 (int, optional): Baud rate timing register 1. Defaults to 0x14 (for 1Mbps).
            mode (int, optional): CAN channel mode (0: normal). Defaults to 0.

        Returns:
            bool: True if successful, False otherwise.
        """
        if not self.can_lib:
            self._log("CAN library not loaded. Cannot open device.")
            return False

        if self.is_open:
            self._log("CAN device already open.")
            return True # Already open and initialized

        # Step 1: Open the physical device
        ret = self.can_lib.VCI_OpenDevice(self.device_type, self.device_idx, 0) # Last param is reserved
        if ret != STATUS_OK:
            self._log(f"Failed to open CAN device. VCI_OpenDevice returned: {ret}")
            return False
        self._log("CAN device opened successfully.")

        # Step 2: Initialize the specific CAN channel
        init_config = VCI_INIT_CONFIG()
        init_config.AccCode = acc_code
        init_config.AccMask = acc_mask
        init_config.Filter = 0  # 0: Receive all frames. 1: Dual filter. 2: Single filter.
        init_config.Timing0 = timing0
        init_config.Timing1 = timing1
        init_config.Mode = mode    # 0: Normal, 1: Loopback, 2: Listen-only (silent)

        ret = self.can_lib.VCI_InitCAN(self.device_type, self.device_idx, self.can_idx, ctypes.byref(init_config))
        if ret != STATUS_OK:
            self._log(f"Failed to initialize CAN channel {self.can_idx}. VCI_InitCAN returned: {ret}")
            self.can_lib.VCI_CloseDevice(self.device_type, self.device_idx) # Clean up opened device
            return False
        self._log(f"CAN channel {self.can_idx} initialized (Timing0={timing0:02X}, Timing1={timing1:02X} -> Baudrate).")

        # Step 3: Start the CAN channel
        ret = self.can_lib.VCI_StartCAN(self.device_type, self.device_idx, self.can_idx)
        if ret != STATUS_OK:
            self._log(f"Failed to start CAN channel {self.can_idx}. VCI_StartCAN returned: {ret}")
            self.can_lib.VCI_CloseDevice(self.device_type, self.device_idx) # Clean up
            return False
        self._log(f"CAN channel {self.can_idx} started successfully.")

        # Clear any old messages from hardware/driver buffers
        self.clear_buffer()

        self.is_open = True
        return True

    def close(self):
        """
        Stops the CAN channel and closes the CAN device.
        """
        if not self.can_lib:
            self._log("CAN library not loaded. Cannot close device.")
            return
        if not self.is_open:
            self._log("CAN device not open or already closed.")
            return

        # Good practice: Stop the CAN channel before closing the device
        # VCI_ResetCAN could also be used, VCI_StopCAN is ZLG specific but often available.
        stop_ret = self.can_lib.VCI_StopCAN(self.device_type, self.device_idx, self.can_idx)
        if stop_ret != STATUS_OK:
            self._log(f"Warning: VCI_StopCAN for channel {self.can_idx} failed with code {stop_ret}, but proceeding to close device.")
        else:
            self._log(f"CAN channel {self.can_idx} stopped.")

        # Close the physical device
        ret = self.can_lib.VCI_CloseDevice(self.device_type, self.device_idx)
        if ret != STATUS_OK:
            self._log(f"Failed to close CAN device. VCI_CloseDevice returned: {ret}")
        else:
            self._log("CAN device closed successfully.")
        self.is_open = False # Update state regardless of VCI_CloseDevice return for safety

    def clear_buffer(self):
        """
        Clears the receive buffer for the specified CAN channel.
        Returns:
            bool: True if successful, False otherwise.
        """
        if not self.can_lib or not self.is_open:
            self._log("CAN device not open. Cannot clear buffer.")
            return False

        ret = self.can_lib.VCI_ClearBuffer(self.device_type, self.device_idx, self.can_idx)
        if ret != STATUS_OK:
            self._log(f"Failed to clear CAN buffer for channel {self.can_idx}. VCI_ClearBuffer returned: {ret}")
            return False
        self._log(f"CAN RX buffer for channel {self.can_idx} cleared.")
        return True


    def send_command(self, motor_id_tx, p_des, v_des, kp, kd, t_ff):
        """
        Packs motor command data using MessagePacker and sends it as a CAN message.

        Args:
            motor_id_tx (int): The CAN ID to which the command should be sent.
            p_des (float): Desired position (radians).
            v_des (float): Desired velocity (rad/s).
            kp (float): Proportional gain.
            kd (float): Derivative gain.
            t_ff (float): Feed-forward torque (Nm).

        Returns:
            bool: True if the command was packed and transmitted successfully, False otherwise.
        """
        if not self.is_open or not self.can_lib:
            self._log("CAN device not open. Cannot send command.")
            return False

        packed_data_bytes = self.packer.pack_cmd(p_des, v_des, kp, kd, t_ff)
        if not packed_data_bytes:
            self._log("Failed to pack command data using MessagePacker.")
            return False

        can_msg = VCI_CAN_OBJ()
        can_msg.ID = motor_id_tx  # Target CAN ID for the motor
        can_msg.SendType = 0      # Normal send
        can_msg.RemoteFlag = 0    # Data frame
        can_msg.ExternFlag = 0    # Standard frame (11-bit ID, common for MIT Cheetah protocol)
        can_msg.DataLen = len(packed_data_bytes)

        # Copy packed data bytes into the Data field of the CAN message structure
        ctypes.memmove(can_msg.Data, packed_data_bytes, len(packed_data_bytes))

        # For debugging the outgoing message:
        # data_hex_str = " ".join([f"{b:02X}" for b in packed_data_bytes])
        # self._log(f"Attempting to send to ID {motor_id_tx:X}: {data_hex_str}")

        # Transmit the CAN frame. The last parameter is the number of frames to send (1 in this case).
        # VCI_Transmit can send multiple frames at once if an array of VCI_CAN_OBJ is passed.
        num_sent = self.can_lib.VCI_Transmit(self.device_type, self.device_idx, self.can_idx,
                                             ctypes.byref(can_msg), 1)

        # ZLG VCI_Transmit typically returns the number of frames successfully sent.
        # Some versions/docs say it returns STATUS_OK (1) on success for single frame, or error.
        # A return of 0 often means buffer full or other error. >0 is generally success.
        # STATUS_ERR (often 0 or 0xFFFFFFFF) indicates definite failure.
        if num_sent > 0 and num_sent != 0xFFFFFFFF: # Assuming >0 is success
            # self._log(f"Successfully sent {num_sent} frame(s) to ID {motor_id_tx:X}.")
            return True
        else:
            self._log(f"Failed to transmit CAN message to ID {motor_id_tx:X}. VCI_Transmit returned: {num_sent}")
            # self._log(f"Message details: ID={can_msg.ID}, DataLen={can_msg.DataLen}, Data={list(can_msg.Data)[:can_msg.DataLen]}")
            return False


    def receive_messages(self, max_frames=10, timeout_ms=20):
        """
        Receives CAN messages from the buffer and unpacks them using MessagePacker.

        Args:
            max_frames (int, optional): Maximum number of frames to attempt to read. Defaults to 10.
            timeout_ms (int, optional): Timeout for the receive operation in milliseconds.
                                       A value of 0 or -1 might mean non-blocking or use default.
                                       DLL behavior can vary. Defaults to 20ms.

        Returns:
            list[tuple]: A list of unpacked data tuples (motor_id, position_rad, velocity_rad_s, torque_nm).
                         Returns an empty list if no messages, timeout, or error.
        """
        if not self.is_open or not self.can_lib:
            self._log("CAN device not open. Cannot receive messages.")
            return []

        # Create a buffer for receiving CAN messages (array of VCI_CAN_OBJ)
        rx_msgs_buffer = (VCI_CAN_OBJ * max_frames)()

        # Check how many frames are actually in the receive buffer first (optional but good practice)
        num_frames_in_hw_buf = self.can_lib.VCI_GetReceiveNum(self.device_type, self.device_idx, self.can_idx)
        if num_frames_in_hw_buf == 0:
            return [] # No messages pending in hardware/driver buffer

        # Determine how many frames to actually try to read
        frames_to_read = min(max_frames, num_frames_in_hw_buf)
        if frames_to_read == 0: # Should not happen if num_frames_in_hw_buf > 0
            return []

        # Attempt to receive the messages
        num_actually_received = self.can_lib.VCI_Receive(self.device_type, self.device_idx, self.can_idx,
                                                         ctypes.byref(rx_msgs_buffer), frames_to_read, timeout_ms)

        if num_actually_received == 0xFFFFFFFF or num_actually_received == STATUS_ERR: # Error
            # self._log(f"Error receiving CAN messages. VCI_Receive returned: {num_actually_received}")
            return []
        if num_actually_received == 0: # Timeout or no messages retrieved despite GetReceiveNum
            # self._log("No CAN messages received (timeout or buffer empty after GetReceiveNum).")
            return []

        # self._log(f"Received {num_actually_received} CAN frames.")
        unpacked_data_list = []
        for i in range(num_actually_received):
            msg = rx_msgs_buffer[i]
            # For debugging raw received frames:
            # self._log(f"Raw RX: ID={msg.ID:X}, Len={msg.DataLen}, Data={' '.join(f'{b:02X}' for b in msg.Data[:msg.DataLen])}")

            # The problem states firmware replies with a fixed CAN_ID (CAN_MASTER_ID).
            # Filtering by this `msg.ID == CAN_MASTER_ID_CONST` could be done here if needed.
            # However, the current design unpacks all valid messages and lets the main GUI/thread
            # filter based on the motor ID extracted from the *payload* by `unpack_reply`.
            # This is flexible if multiple motors reply on the same CAN_MASTER_ID but have different payload IDs.

            data_bytes = bytes(list(msg.Data)[:msg.DataLen]) # Convert ctypes array to Python bytes
            unpacked_reply = self.packer.unpack_reply(data_bytes)

            if unpacked_reply:
                # unpacked_reply is (motor_id_from_payload, p_rad, v_rad_s, t_nm)
                unpacked_data_list.append(unpacked_reply)
            else:
                self._log(f"Failed to unpack message from CAN ID {msg.ID:X}, Data: {data_bytes.hex()}")

        return unpacked_data_list

    def __del__(self):
        """
        Destructor to ensure the CAN device is closed when the CANHandler object is deleted.
        """
        if self.is_open:
            self.close()

# --- Example Usage (for direct testing of this module) ---
if __name__ == '__main__':
    import sys
    import time # For sleep in test

    # A simple logger class for console output during testing
    class SimpleTestLogger:
        def log(self, message):
            print(message)

    logger = SimpleTestLogger()

    # Attempt to find ControlCAN.dll automatically for testing convenience
    # This simplified logic is for the __main__ block; the class has more robust path checking.
    dll_name_test = "ControlCAN.dll"
    script_dir_test = os.path.dirname(os.path.abspath(__file__))

    # Potential paths for the DLL when testing
    # 1. In the same directory as this script (test/can_communication/)
    dll_local_path_test = os.path.join(script_dir_test, dll_name_test)
    # 2. In the parent 'test' directory (test/)
    dll_parent_test_path = os.path.join(script_dir_test, "..", dll_name_test)
    # 3. In 'test/' relative to CWD, if CWD is project root
    dll_project_root_test_path = os.path.join(os.getcwd(), "test", dll_name_test)

    can_dll_path_test = ""
    if os.path.exists(dll_local_path_test):
        can_dll_path_test = dll_local_path_test
    elif os.path.exists(dll_parent_test_path):
        can_dll_path_test = dll_parent_test_path
    elif os.path.exists(dll_project_root_test_path):
        can_dll_path_test = dll_project_root_test_path
    else:
        can_dll_path_test = dll_name_test # Fallback: assume it's in PATH or CWD
        logger.log(f"Warning: {dll_name_test} not found in typical test locations relative to script. "
                   f"Attempting to load '{dll_name_test}' (may rely on system PATH or CWD).")

    # --- Test Parameters ---
    # Assumed CAN ID for the motor we are trying to command (verify with firmware)
    motor_id_to_test = 0x01
    # Baud rate for testing (1Mbps is common for motor control)
    # ZLG Timing Registers for 1Mbps: Timing0=0x00, Timing1=0x14
    test_baud_timing0 = 0x00
    test_baud_timing1 = 0x14

    print(f"\n--- CANHandler Test Utility ---")
    print(f"Using DLL path: {os.path.abspath(can_dll_path_test) if os.path.exists(can_dll_path_test) else can_dll_path_test}")
    print(f"Target motor CAN ID for commands: {motor_id_to_test:X}")
    print(f"Baud rate settings: Timing0={test_baud_timing0:02X}, Timing1={test_baud_timing1:02X} (expected 1Mbps)")

    # Instantiate CANHandler
    # Note: VCI_USBCAN2 is a common device_type for CANalyst-II. Adjust if using a different model.
    handler = CANHandler(dll_path=can_dll_path_test, device_type=VCI_USBCAN2, logger=logger)

    if not handler.can_lib:
        logger.log("Failed to load ControlCAN library. CANHandler test cannot proceed.")
        sys.exit(1)

    logger.log(f"\nAttempting to open CAN device "
               f"(Type: {handler.device_type}, DeviceIdx: {handler.device_idx}, CANIdx: {handler.can_idx})...")

    if handler.open(timing0=test_baud_timing0, timing1=test_baud_timing1):
        logger.log("CAN device opened successfully.")

        # Example: Send a "hold position at 0" command
        # Parameters for the command: p_des=0 rad, v_des=0 rad/s, kp=10, kd=0.1, t_ff=0 Nm
        # These are example values; adjust Kp/Kd based on your motor setup for safe testing.
        # Start with Kp=0, Kd=0, T_ff=0 if unsure, to avoid unexpected motion.
        p_des_example = 0.0  # radians
        v_des_example = 0.0  # rad/s
        kp_example = 0.0     # Nm/rad (Start with 0 for safety)
        kd_example = 0.0     # Nm/(rad/s) (Start with 0 for safety)
        t_ff_example = 0.0   # Nm

        logger.log(f"\nSending test command to motor ID {motor_id_to_test:X}: "
                   f"P={p_des_example:.2f}, V={v_des_example:.2f}, Kp={kp_example:.1f}, Kd={kd_example:.2f}, Tff={t_ff_example:.2f}")

        sent_ok = handler.send_command(motor_id_to_test, p_des_example, v_des_example,
                                       kp_example, kd_example, t_ff_example)

        if sent_ok:
            logger.log(f"Test command sent successfully to motor ID {motor_id_to_test:X}.")

            logger.log("\nWaiting briefly for potential replies (e.g., 100ms)...")
            time.sleep(0.1)

            logger.log("Attempting to receive messages...")
            received_replies = handler.receive_messages(max_frames=5) # Try to get up to 5 frames

            if received_replies:
                logger.log(f"Received {len(received_replies)} replies:")
                for i, reply_tuple in enumerate(received_replies):
                    # reply_tuple is (motor_id_from_payload, p_rad, v_rad_s, t_nm)
                    r_id, r_p, r_v, r_t = reply_tuple
                    logger.log(f"  Reply {i+1}: MotorID(payload)={r_id}, Pos={r_p:.3f} rad, "
                               f"Vel={r_v:.3f} rad/s, Torque={r_t:.3f} Nm")
            else:
                logger.log("No replies received for the test command (or timed out).")
        else:
            logger.log(f"Failed to send test command to motor ID {motor_id_to_test:X}.")

        logger.log("\nClosing CAN device...")
        handler.close()
    else:
        logger.log("Failed to open CAN device. Check hardware connection and DLL compatibility.")

    logger.log("\nCANHandler test finished.")
