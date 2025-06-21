import numpy as np

# Angle limits for GUI and motor control (degrees)
ANGLE_MIN_DEG = -120.0
ANGLE_MAX_DEG = 0.0

# Conversion factor
DEG_TO_RAD = np.pi / 180.0
RAD_TO_DEG = 180.0 / np.pi

# CAN related (placeholder, actual values might be needed by CAN handler or GUI)
# Example: Default motor ID to command if only one is used
DEFAULT_MOTOR_ID = 1
# Example: CAN Baud rate settings (if configurable by GUI, though likely fixed in CANHandler)
# BAUD_RATE_1MBPS_TIMING = (0x00, 0x14)
# BAUD_RATE_500KBPS_TIMING = (0x00, 0x1C)

# Default PD gains (can be changed by user via GUI)
DEFAULT_KP = 10.0  # N-m/rad
DEFAULT_KD = 0.2   # N-m/(rad/s)
DEFAULT_T_FF = 0.0 # Nm

# Slider precision / steps
# For a -120 to 0 degree slider, if we want 0.1 degree precision:
# Range = 120 degrees. Steps = 120 / 0.1 = 1200 steps.
SLIDER_ANGLE_STEPS = 1200

# Automated Cycle Test Defaults
DEFAULT_CYCLE_COUNT = 5
DEFAULT_ACCELERATION = 2.0  # rad/s^2 (example value, needs tuning for smooth motion)
# Max velocity for cycle test can also be a constant or calculated based on accel and distance
DEFAULT_CYCLE_MAX_VELOCITY = np.pi # rad/s (approx 180 deg/s or 0.5 full cycle per second)

# pyqtgraph plot update rate (ms)
PLOT_UPDATE_MS = 50 # Update plot every 50ms (20 Hz refresh rate for graphs)

# High-frequency CAN command send interval (ms) for keep-alive and smooth control
CAN_SEND_INTERVAL_MS = 20 # Send CAN command every 20ms (50 Hz command rate)

# Colors for pyqtgraph plots
TARGET_ANGLE_COLOR = 'r' # Red for target angle curve
ACTUAL_ANGLE_COLOR = 'b' # Blue for actual angle curve

# Serial port communication parameters
DEFAULT_BAUDRATE = 115200 # Standard baud rate for serial communication

# Logger configuration
MAX_LOG_LINES = 200 # Maximum number of lines to keep in the GUI debug log

# UI Refresh rates for various display elements (in milliseconds)
DISPLAY_PANEL_REFRESH_MS = 100 # How often to update numerical text displays (e.g., angle, speed, torque)

# Joint Diagram graphical parameters (in pixels or relative units)
THIGH_LENGTH = 100      # Length of the thigh segment in the diagram
SHANK_LENGTH = 100      # Length of the shank segment in the diagram
JOINT_ORIGIN_X = 150    # X-coordinate of the hip joint (origin for drawing)
JOINT_ORIGIN_Y = 100    # Y-coordinate of the hip joint
THIGH_COLOR = (100, 100, 100) # RGB color for the thigh segment (Gray)
SHANK_COLOR = (150, 150, 150) # RGB color for the shank segment (Light Gray)
JOINT_COLOR = (255, 0, 0)     # RGB color for highlighting joints (Red) - currently unused, hip/knee have own colors
HIP_COLOR = (0,0,0)           # RGB color for the hip joint articulation point (Black)
KNEE_COLOR = (0,0,0)          # RGB color for the knee joint articulation point (Black)

# If ControlCAN.dll path needs to be globally accessible (though CANHandler tries to find it)
# CONTROL_CAN_DLL_PATH = "ControlCAN.dll" # or "test/ControlCAN.dll" when running from root
# It's better if CANHandler manages its own DLL path finding.

# Message packer constants (already in message_packer.py, but good to be aware of them)
# P_MIN, P_MAX = -12.5, 12.5  # rad
# V_MIN, V_MAX = -65.0, 65.0  # rad/s
# T_MIN, T_MAX = -18.0, 18.0  # Nm
# KP_MIN, KP_MAX = 0.0, 500.0 # Theoretical min/max for Kp gain (from message_packer or firmware spec)
# KD_MIN, KD_MAX = 0.0, 5.0   # Theoretical min/max for Kd gain

# Practical display and input ranges for GUI parameter tuning dials/spinboxes
# These should be within the firmware's operational limits and the message packing limits.
PARAM_KP_MIN_DISPLAY = 0.0      # Min Kp value settable in GUI
PARAM_KP_MAX_DISPLAY = 100.0    # Max Kp value settable in GUI (practical upper limit)
PARAM_KD_MIN_DISPLAY = 0.0      # Min Kd value settable in GUI
PARAM_KD_MAX_DISPLAY = 5.0      # Max Kd value settable in GUI (practical upper limit)
PARAM_T_FF_MIN_DISPLAY = -5.0   # Min Feed-forward torque value settable in GUI (Nm)
PARAM_T_FF_MAX_DISPLAY = 5.0    # Max Feed-forward torque value settable in GUI (Nm)

# Number of steps/resolution for parameter adjustment Dials in the GUI
# Example: For Kp from 0 to 100 with 0.1 resolution, steps = 100 / 0.1 = 1000
KP_DIAL_STEPS = int((PARAM_KP_MAX_DISPLAY - PARAM_KP_MIN_DISPLAY) / 0.1) # 0.1 resolution for Kp dial
KD_DIAL_STEPS = int((PARAM_KD_MAX_DISPLAY - PARAM_KD_MIN_DISPLAY) / 0.01) # 0.01 resolution for Kd dial
T_FF_DIAL_STEPS = int((PARAM_T_FF_MAX_DISPLAY - PARAM_T_FF_MIN_DISPLAY) / 0.05) # 0.05 resolution for T_ff dial

# Automated Cycle Test: Define the two extreme target positions in radians
CYCLE_POS_A_RAD = ANGLE_MAX_DEG * DEG_TO_RAD # Typically 0.0 rad (e.g., straight leg)
CYCLE_POS_B_RAD = ANGLE_MIN_DEG * DEG_TO_RAD # Typically -120 deg converted to rad (e.g., max flexion)

# CAN Communication Identifiers
# MOTOR_CMD_CAN_ID: The CAN ID to which commands for the motor are sent.
# This might be specific to the motor (e.g., motor 1 = ID 0x01, or a manufacturer-defined ID like 0x141).
# The problem statement mentions "固件通过CAN总线接收实时的PD控制指令". This ID is for that.
MOTOR_CMD_CAN_ID = 0x01  # Default CAN ID for sending commands to the motor (e.g., Motor 1).
                         # This needs to be verified against the GIM6010 firmware documentation.
                         # Common GIM6010 IDs are 0x140 + motor_number.

# MOTOR_REPLY_CAN_ID: The CAN ID expected for replies from the motor.
# The problem states "并以固定的CAN_ID（CAN_MASTER_ID）回复电机状态."
# This implies the motor replies using a fixed CAN frame ID, irrespective of its own specific ID.
# This CAN_MASTER_ID could be 0x00, or some other value defined by the firmware.
# The CANHandler currently unpacks all valid messages; the MainWindow filters by payload ID.
# If strict CAN frame ID filtering is needed in CANHandler, this constant would be used there.
# For now, this is informational. The application primarily uses the motor ID from the message payload.
# MOTOR_REPLY_CAN_ID = 0x00 # Example: if firmware always replies with CAN ID 0x00. (Currently unused for filtering in CANHandler)


# File name for the ZLG CANalyst-II driver DLL
DLL_NAME = "ControlCAN.dll" # CANHandler will search for this file.

# Application Information
APP_NAME = "Exoskeleton Motor GUI"    # Application Name
APP_VERSION = "0.1.0"                 # Application Version
