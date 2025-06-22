import sys
import time
import numpy as np
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QSplitter, QMessageBox, QApplication)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer, QSettings

# --- Import GUI Widgets ---
from test.gui.widgets.control_panel import ControlPanel
from test.gui.widgets.motion_panel import MotionPanel
from test.gui.widgets.param_panel import ParamPanel
from test.gui.widgets.display_panel import DisplayPanel

# --- Import Communication Handlers ---
from test.serial_communication.serial_handler import SerialHandler
from test.can_communication.can_handler import CANHandler
# from test.can_communication.message_packer import MessagePacker # Not directly used in MW, but by CANHandler

# --- Import Utilities ---
from test.utils import constants as const
from test.utils.logger import GUILogger


class CANCommunicationThread(QThread):
    """
    Manages CAN communication in a separate thread to prevent GUI freezing.
    It periodically sends commands based on current target parameters and
    receives and emits motor status data.
    """
    # --- Signals ---
    # Emits: (motor_id, position_rad, velocity_rad_s, torque_nm)
    dataReceived = pyqtSignal(int, float, float, float)
    # Emits: status or error messages related to CAN communication
    canStatusMessage = pyqtSignal(str)
    # Emitted when the thread's run() method finishes
    finished = pyqtSignal()

    def __init__(self, can_handler, command_can_id, initial_params, parent=None):
        """
        Initializes the CAN communication thread.
        Args:
            can_handler (CANHandler): Instance of the CANHandler for CAN operations.
            command_can_id (int): The CAN ID to which commands will be sent.
            initial_params (tuple): Initial (kp, kd, t_ff) values.
            parent (QObject, optional): Parent QObject. Defaults to None.
        """
        super().__init__(parent)
        self.can_handler = can_handler
        self.command_can_id = command_can_id
        self._running = False # Flag to control the main loop
        # If True, GUI-initiated updates to P, Kp, Kd are ignored.
        # Used by cycle test or E-Stop to take over parameter control.
        self._gui_updates_locked = False

        # --- Current command parameters (will be updated by main GUI or cycle test) ---
        self.p_des_rad = 0.0    # Desired position (radians)
        self.v_des_rad_s = 0.0  # Desired velocity (rad/s), typically 0 for position hold
        self.kp = initial_params[0] # Proportional gain
        self.kd = initial_params[1] # Derivative gain
        self.t_ff_nm = initial_params[2] # Feed-forward torque

        # QTimer for periodic command sending
        self.send_timer = QTimer()
        self.send_timer.moveToThread(self) # Critical: Ensure timer lives in this thread
        self.send_timer.timeout.connect(self._send_can_command)


    def run(self):
        """Main execution loop for the thread."""
        self._running = True
        if not self.can_handler or not self.can_handler.is_open:
            self.canStatusMessage.emit("CAN device not open. CANCommunicationThread cannot start.")
            self._running = False
            self.finished.emit() # Signal that thread is done
            return

        self.canStatusMessage.emit("CANCommunicationThread started.")
        self.send_timer.start(const.CAN_SEND_INTERVAL_MS) # Start periodic command sender

        while self._running:
            if not self.can_handler.is_open: # Check if CAN device was closed externally
                self.canStatusMessage.emit("CAN device closed unexpectedly. Stopping CANCommunicationThread.")
                break

            # Receive messages from CAN bus
            received_data_list = self.can_handler.receive_messages(max_frames=5, timeout_ms=10) # Short non-blocking read

            for data_tuple in received_data_list:
                # data_tuple is (motor_id, p_rad, v_rad_s, t_nm)
                self.dataReceived.emit(*data_tuple) # Emit received data

            # QThread.msleep(5) # Small delay to yield; adjust if needed.
            # Current design relies on receive_messages timeout and send_timer interval.

        self.send_timer.stop() # Stop the command sending timer
        self.canStatusMessage.emit("CANCommunicationThread finished.")
        self.finished.emit() # Signal completion

    def _send_can_command(self):
        """Periodically called by send_timer to send the current command packet."""
        # This method sends the current state of self.p_des_rad, self.kp, etc.
        # It always sends if the thread is running and CAN is open.
        # The _gui_updates_locked flag controls who can *change* these parameters from the GUI.
        if self._running and self.can_handler and self.can_handler.is_open:
            success = self.can_handler.send_command(
                self.command_can_id,
                self.p_des_rad, self.v_des_rad_s,
                self.kp, self.kd, self.t_ff_nm
            )
            if not success:
                self.canStatusMessage.emit(f"Error: Failed to send CAN command to ID {self.command_can_id:X}")

    # --- Methods to update command parameters (called from main GUI thread) ---
    def update_target_position(self, p_des_rad, v_des_rad_s=0.0):
        """Called by GUI (e.g., slider) to update target position and velocity."""
        if not self._gui_updates_locked:
            self.p_des_rad = p_des_rad
            self.v_des_rad_s = v_des_rad_s
        # else:
            # self.canStatusMessage.emit("CAN thread: GUI update for target position ignored (locked).")

    def update_pd_gains(self, kp, kd):
        """Called by GUI (e.g., dials) to update PD gains."""
        if not self._gui_updates_locked:
            self.kp = kp
            self.kd = kd
        # else:
            # self.canStatusMessage.emit("CAN thread: GUI update for PD gains ignored (locked).")

    def update_feedforward_torque(self, t_ff_nm):
        """Called by GUI (e.g., dial) to update feedforward torque."""
        if not self._gui_updates_locked:
            self.t_ff_nm = t_ff_nm
        # else:
            # self.canStatusMessage.emit("CAN thread: GUI update for T_ff ignored (locked).")

    def update_all_params(self, p_des_rad, v_des_rad_s, kp, kd, t_ff_nm):
        """
        Called by automated processes (like CycleTestThread or E-Stop)
        to directly set all parameters, bypassing the GUI lock.
        """
        self.p_des_rad = p_des_rad
        self.v_des_rad_s = v_des_rad_s
        self.kp = kp
        self.kd = kd
        self.t_ff_nm = t_ff_nm

    def stop(self):
        """Signals the thread to stop its execution loop."""
        self._running = False

    def set_gui_updates_locked(self, locked):
        """
        Controls whether updates from GUI controls (slider, dials) are applied.
        If True, GUI updates are ignored. Automated sequences (cycle test, E-Stop)
        can still update params via update_all_params().
        Args:
            locked (bool): True to lock GUI updates, False to unlock.
        """
        self._gui_updates_locked = locked
        if locked:
            self.canStatusMessage.emit("CAN thread: GUI parameter updates are now LOCKED.")
        else:
            self.canStatusMessage.emit("CAN thread: GUI parameter updates are now UNLOCKED.")


class CycleTestThread(QThread):
    """
    Executes an automated cycle test in a separate thread.
    It generates a trajectory (trapezoidal/triangular velocity profile) and emits
    CAN command parameters at regular intervals for the CANCommunicationThread to send.
    """
    # --- Signals ---
    # Emits: (p_rad, v_rad_s, kp, kd, t_ff_nm) for the CANCommunicationThread
    newCANCommandParams = pyqtSignal(float, float, float, float, float)
    # Emits: status messages about the cycle test progress
    cycleStatusUpdate = pyqtSignal(str)
    # Emitted when the cycle test completes or is stopped
    cycleFinished = pyqtSignal()

    def __init__(self, num_cycles, accel, max_vel, initial_kp, initial_kd, parent=None):
        """
        Initializes the CycleTestThread.
        Args:
            num_cycles (int): Number of full cycles (A->B->A) to perform.
            accel (float): Acceleration in rad/s^2 for the trajectory.
            max_vel (float): Maximum velocity in rad/s for the trajectory.
            initial_kp (float): Kp gain to use during the cycle test.
            initial_kd (float): Kd gain to use during the cycle test.
            parent (QObject, optional): Parent QObject. Defaults to None.
        """
        super().__init__(parent)
        self.num_cycles = num_cycles
        self.accel_rad_s2 = abs(accel) # Ensure positive acceleration
        self.max_vel_rad_s = abs(max_vel) # Ensure positive max velocity

        self.kp = initial_kp
        self.kd = initial_kd
        self.t_ff_nm = 0.0 # Feed-forward torque is typically 0 for simple cycle tests

        # Define cycle endpoints from constants
        self.pos_A_rad = const.CYCLE_POS_A_RAD # e.g., 0.0 rad (fully extended)
        self.pos_B_rad = const.CYCLE_POS_B_RAD # e.g., -120 deg in rad (max flexion)

        self._running = False # Flag to control the main loop
        self._current_actual_pos_rad = self.pos_A_rad # Assume starting at Pos A for any feedback (currently unused by planner)

    def update_actual_position(self, pos_rad):
        """
        Slot to receive the current actual motor position from the main GUI.
        Currently not used by the trajectory planner itself (which is open-loop target generation),
        but could be used for logging or more advanced feedback within the cycle test.
        Args:
            pos_rad (float): Actual motor position in radians.
        """
        self._current_actual_pos_rad = pos_rad

    def run(self):
        """Main execution loop for the cycle test."""
        self._running = True
        self.cycleStatusUpdate.emit(f"Cycle test started: {self.num_cycles} cycles, "
                                    f"Accel={self.accel_rad_s2:.2f} rad/s^2, MaxVel={self.max_vel_rad_s:.2f} rad/s.")

        # Time step for trajectory generation, should match CAN send interval for smoothness
        dt_sec = float(const.CAN_SEND_INTERVAL_MS) / 1000.0

        for i in range(self.num_cycles):
            if not self._running: break # Check for stop signal before each segment
            self.cycleStatusUpdate.emit(f"Cycle {i+1}/{self.num_cycles}: Moving A ({self.pos_A_rad:.2f} rad) -> B ({self.pos_B_rad:.2f} rad)")
            self._execute_move(self.pos_A_rad, self.pos_B_rad, dt_sec)

            if not self._running: break
            # Optional: Short pause between movements (e.g., QThread.msleep(200))

            if not self._running: break
            self.cycleStatusUpdate.emit(f"Cycle {i+1}/{self.num_cycles}: Moving B ({self.pos_B_rad:.2f} rad) -> A ({self.pos_A_rad:.2f} rad)")
            self._execute_move(self.pos_B_rad, self.pos_A_rad, dt_sec)

            if not self._running: break
            # Optional: Short pause

        if self._running: # Test completed all cycles
            self.cycleStatusUpdate.emit("Cycle test completed successfully.")
        else: # Test was stopped prematurely
            self.cycleStatusUpdate.emit("Cycle test stopped by user.")

        self._running = False
        self.cycleFinished.emit() # Signal completion

    def _execute_move(self, start_pos_rad, end_pos_rad, dt_sec):
        """
        Generates and emits trajectory points for a single segment of movement
        (e.g., A to B or B to A) using a trapezoidal or triangular velocity profile.
        Args:
            start_pos_rad (float): Starting position in radians.
            end_pos_rad (float): Ending position in radians.
            dt_sec (float): Time step for emitting trajectory points, in seconds.
        """
        distance = abs(end_pos_rad - start_pos_rad)
        if distance < 1e-6: # Negligible distance, no move needed
            self.newCANCommandParams.emit(end_pos_rad, 0.0, self.kp, self.kd, self.t_ff_nm)
            return

        direction = np.sign(end_pos_rad - start_pos_rad)

        # Calculate time to accelerate to max_vel_rad_s and distance covered
        t_accel = self.max_vel_rad_s / self.accel_rad_s2
        d_accel = 0.5 * self.accel_rad_s2 * t_accel**2

        if d_accel * 2 >= distance: # Triangular profile (doesn't reach max_vel_rad_s)
            t_accel = np.sqrt(distance / self.accel_rad_s2) # Time to reach midpoint
            t_const_vel = 0.0
            effective_max_vel = self.accel_rad_s2 * t_accel # Actual peak velocity reached
        else: # Trapezoidal profile
            d_const_vel = distance - 2 * d_accel
            t_const_vel = d_const_vel / self.max_vel_rad_s
            effective_max_vel = self.max_vel_rad_s

        t_total_segment = 2 * t_accel + t_const_vel # Total time for this segment

        current_time_in_segment = 0.0
        current_pos_rad = start_pos_rad
        current_vel_rad_s = 0.0

        while current_time_in_segment < t_total_segment and self._running:
            # Determine current phase: acceleration, constant velocity, or deceleration
            if current_time_in_segment < t_accel: # Acceleration phase
                current_vel_rad_s = direction * self.accel_rad_s2 * current_time_in_segment
                current_pos_rad = start_pos_rad + direction * (0.5 * self.accel_rad_s2 * current_time_in_segment**2)
            elif current_time_in_segment < t_accel + t_const_vel: # Constant velocity phase
                current_vel_rad_s = direction * effective_max_vel
                # Position at end of accel phase: start_pos_rad + direction * d_accel
                # Time into constant velocity phase: current_time_in_segment - t_accel
                current_pos_rad = (start_pos_rad + direction * d_accel) + \
                                  current_vel_rad_s * (current_time_in_segment - t_accel)
            else: # Deceleration phase
                time_into_decel = current_time_in_segment - (t_accel + t_const_vel)
                current_vel_rad_s = direction * (effective_max_vel - self.accel_rad_s2 * time_into_decel)
                # Position calculation for deceleration phase (integrating from end or start of decel)
                # Pos at start of decel: end_pos_rad - direction * d_accel (if looking from end)
                # More robust: calculate remaining time and integrate backwards from end_pos_rad
                time_remaining_in_decel = t_total_segment - current_time_in_segment
                current_pos_rad = end_pos_rad - direction * (0.5 * self.accel_rad_s2 * time_remaining_in_decel**2)

            # Ensure velocity doesn't overshoot due to discrete steps, especially near zero at end of decel
            if (direction > 0 and current_vel_rad_s < 0 and current_time_in_segment > t_accel + t_const_vel) or \
               (direction < 0 and current_vel_rad_s > 0 and current_time_in_segment > t_accel + t_const_vel):
                current_vel_rad_s = 0.0 # Clamp to zero if overshot

            # Clamp position to target to avoid overshooting due to numerical precision
            if direction > 0: current_pos_rad = min(current_pos_rad, end_pos_rad)
            else: current_pos_rad = max(current_pos_rad, end_pos_rad)

            # Emit the calculated parameters for the CAN command
            self.newCANCommandParams.emit(current_pos_rad, current_vel_rad_s, self.kp, self.kd, self.t_ff_nm)

            QThread.msleep(int(dt_sec * 1000)) # Wait for one time step
            current_time_in_segment += dt_sec

        # Ensure final position is commanded precisely if loop finished normally
        if self._running:
            self.newCANCommandParams.emit(end_pos_rad, 0.0, self.kp, self.kd, self.t_ff_nm)

    def stop(self):
        """Signals the thread to stop its execution loop."""
        self._running = False


class MainWindow(QMainWindow):
    """
    The MainWindow class for the Exoskeleton Motor Control GUI.
    It orchestrates the various UI panels, communication handlers (Serial, CAN),
    and data processing threads.
    """
    def __init__(self):
        """Initializes the main application window, components, and connections."""
        super().__init__()
        self.setWindowTitle(f"{const.APP_NAME} v{const.APP_VERSION}")
        # Set a reasonable default size and position for the window
        self.setGeometry(100, 100, 1280, 768)

        # Initialize core components:
        # Logger: For displaying messages in the GUI's debug area.
        self.logger = GUILogger(max_lines=const.MAX_LOG_LINES)
        # Serial Handler: Manages serial port communication.
        self.serial_handler = SerialHandler(logger=self.logger)
        # CAN Handler: Manages CAN bus communication via ControlCAN.dll.
        # The DLL path is specified here; CANHandler will attempt to locate and load it.
        self.can_handler = CANHandler(dll_path=const.DLL_NAME, logger=self.logger)

        # Thread for handling CAN communication (sending commands, receiving data)
        self.can_comm_thread = None
        # Thread for executing automated cycle tests
        self.cycle_test_thread = None

        # --- Internal state variables for control parameters ---
        self._current_kp = const.DEFAULT_KP                # Current Proportional Gain
        self._current_kd = const.DEFAULT_KD                # Current Derivative Gain
        self._current_t_ff = const.DEFAULT_T_FF            # Current Feed-forward Torque
        self._current_target_angle_deg = const.ANGLE_MAX_DEG # Current target angle from GUI (degrees)
        self._last_actual_angle_deg = 0.0                  # Last known actual angle from motor (degrees)

        self._plot_start_time = time.time() # Time reference for real-time plotting
        # QSettings for saving/loading window geometry and other preferences
        self._settings = QSettings("ExoDevCompany", const.APP_NAME) # Org name, App name

        self._init_ui()          # Setup the user interface layout and panels
        self._connect_signals()  # Connect signals from UI elements to handler methods
        self._load_settings()    # Load previously saved window settings

        # Trigger an initial refresh of available COM ports when the application starts
        self.control_panel.refreshPortsClicked.emit()


    def _init_ui(self):
        """Initializes the UI layout, creates all panels, and sets up the central widget."""
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        main_layout = QHBoxLayout(self.central_widget) # Main layout is horizontal

        # Create instances of all UI panels
        self.control_panel = ControlPanel()    # For serial port and system commands
        self.param_panel = ParamPanel()      # For CAN connection and parameter tuning
        self.motion_panel = MotionPanel()      # For real-time motion control and cycle tests
        self.display_panel = DisplayPanel()    # For numerical, graphical, and log displays

        # Connect the application's logger to the debug QTextEdit in the DisplayPanel
        self.logger.connect_to_widget(self.display_panel.get_debug_text_edit())
        self.logger.log(f"{const.APP_NAME} v{const.APP_VERSION} started. Python: {sys.version.split()[0]}")

        # --- Arrange panels using a QSplitter for resizable sections ---
        # Left section: ControlPanel and ParamPanel
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.control_panel)
        left_layout.addWidget(self.param_panel)
        left_layout.addStretch(1) # Add stretch to push panels to the top
        left_widget = QWidget()
        left_widget.setLayout(left_layout)

        # Right section: MotionPanel and DisplayPanel
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.motion_panel)
        right_layout.addWidget(self.display_panel, 1) # DisplayPanel takes more vertical space
        right_widget = QWidget()
        right_widget.setLayout(right_layout)

        # Horizontal splitter to divide left and right sections
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        # Set initial sizes for the splitter sections (can be adjusted by user)
        # These are approximate proportions; actual pixel sizes depend on window size.
        total_width = self.width()
        splitter.setSizes([int(total_width * 0.3), int(total_width * 0.7)])

        main_layout.addWidget(splitter)

    def _connect_signals(self):
        """Connects signals from UI panels to their respective handler methods in MainWindow."""
        # --- Control Panel (Serial Communication) Signals ---
        self.control_panel.refreshPortsClicked.connect(self._update_com_ports)
        self.control_panel.connectSerialClicked.connect(self._connect_serial)
        self.control_panel.disconnectSerialClicked.connect(self._disconnect_serial)
        self.control_panel.enableMotorClicked.connect(self._enable_motor)
        self.control_panel.disableMotorClicked.connect(self._disable_motor_serial)
        self.control_panel.setZeroPointClicked.connect(self._set_zero_point)
        self.control_panel.emergencyStopClicked.connect(self._emergency_stop)

        # --- Parameter Panel (CAN Connection & PD/Tff Gains) Signals ---
        self.param_panel.connectCANClicked.connect(self._connect_can)
        self.param_panel.disconnectCANClicked.connect(self._disconnect_can)
        self.param_panel.parametersChanged.connect(self._update_pd_parameters)

        # --- Motion Panel (CAN-based Motion Control) Signals ---
        self.motion_panel.targetAngleChanged.connect(self._update_target_angle)
        self.motion_panel.startCycleTest.connect(self._start_cycle_test)
        self.motion_panel.stopCycleTest.connect(self._stop_cycle_test)

        # DisplayPanel updates are handled programmatically by methods like _handle_can_data.

    # --- Serial Control Slot Methods ---
    def _update_com_ports(self):
        """Handles the request to refresh and list available COM ports."""
        self.logger.log("Refreshing COM ports...")
        ports = self.serial_handler.get_available_ports()
        if ports:
            self.control_panel.populate_ports(ports)
            self.logger.log(f"Available serial ports found: {', '.join(ports)}")
        else:
            self.control_panel.populate_ports([]) # Clear list if no ports
            self.logger.log("No COM ports found.")

    def _connect_serial(self, port_name):
        """Handles connecting to the selected serial port."""
        self.logger.log(f"Attempting to connect to serial port: {port_name}...")
        if self.serial_handler.connect(port_name, baudrate=const.DEFAULT_BAUDRATE):
            self.control_panel.set_connection_status(True) # Update button states etc.
            self.logger.log(f"Successfully connected to serial port {port_name}.")
        else:
            self.control_panel.set_connection_status(False)
            QMessageBox.warning(self, "Serial Connection Error",
                                f"Failed to connect to {port_name}. Check device and port settings.")
            self.logger.log(f"Error: Failed to connect to serial port {port_name}.")

    def _disconnect_serial(self):
        """Handles disconnecting from the current serial port."""
        self.serial_handler.disconnect()
        self.control_panel.set_connection_status(False) # Update UI state
        self.logger.log("Disconnected from serial port.")

    def _enable_motor(self):
        """Sends the 'enable motor' command via serial ('m') AND the CAN 'Enter Motor Mode' command."""
        if not (self.serial_handler.ser and self.serial_handler.ser.is_open):
            self.logger.log("Cannot enable motor: Serial port not connected.")
            QMessageBox.warning(self, "Serial Error", "Serial port not connected. Cannot perform serial part of enable sequence.")
            return

        self.logger.log("Step 1: Sending 'Enable Motor' (m) command via serial...")
        serial_success = self.serial_handler.enable_motor() # Assuming this returns True on success or similar

        if not serial_success:
            self.logger.log("Failed to send serial 'm' command or serial port error during enable.")
            QMessageBox.warning(self, "Serial Error", "Failed to send 'm' command via serial. Motor not enabled.")
            return

        self.logger.log("Serial 'm' command sent successfully.")

        # Proceed to CAN command only if serial was successful
        if not (self.can_handler and self.can_handler.is_open):
            self.logger.log("Cannot complete motor enable: CAN is not connected/open. Serial 'm' was sent, but motor may not be fully operational without CAN 'Enter Motor Mode' command.")
            QMessageBox.warning(self, "CAN Error", "CAN is not connected. Serial 'm' sent, but 'Enter Motor Mode' CAN command cannot be sent. Motor might not be fully active.")
            return

        self.logger.log(f"Step 2: Sending CAN 'Enter Motor Mode' command to ID {const.MOTOR_CMD_CAN_ID:X}...")
        # Data for "Enter Motor Mode" command: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        enter_motor_mode_data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]

        # Using the new send_raw_command method from can_handler
        # Parameters for send_raw_command: motor_id_tx, raw_data, extern_flag=0, remote_flag=0, send_type=0
        # For MIT Cheetah, it's typically standard frame (ExternFlag=0) and data frame (RemoteFlag=0).
        can_success = self.can_handler.send_raw_command(
            motor_id_tx=const.MOTOR_CMD_CAN_ID,
            raw_data=enter_motor_mode_data,
            extern_flag=0, # Standard CAN ID
            remote_flag=0  # Data frame
        )

        if can_success:
            self.logger.log("CAN 'Enter Motor Mode' command sent successfully. Motor should now be fully enabled.")
            QMessageBox.information(self, "Motor Enabled", "Motor enable sequence (Serial 'm' + CAN 'Enter Mode') completed successfully.")
        else:
            self.logger.log("Failed to send CAN 'Enter Motor Mode' command. Serial 'm' was sent, but motor may not be fully operational.")
            QMessageBox.warning(self, "CAN Command Error", "Failed to send 'Enter Motor Mode' CAN command. Motor might not be fully active.")

    def _disable_motor_serial(self):
        """Sends the 'disable motor' (ESC) command via serial."""
        if self.serial_handler.ser and self.serial_handler.ser.is_open:
            self.logger.log("Sending 'Disable Motor' (ESC) command via serial...")
            self.serial_handler.disable_motor()
            # Important: If CAN is active, this serial command should ideally also cause the firmware
            # to stop responding to CAN commands or enter a safe state.
            # The GUI should also reflect this, e.g., by stopping active CAN threads or commands.
            if self.can_comm_thread and self.can_comm_thread.isRunning():
                 self.logger.log("Serial disable sent. Assuming firmware also handles CAN motor stop.")
                 # For a more robust system, one might also explicitly send safe CAN commands here,
                 # like setting Kp/Kd to 0, as done in _emergency_stop.
        else:
            self.logger.log("Cannot disable motor: Serial port not connected.")
            QMessageBox.warning(self, "Serial Error", "Serial port not connected. Cannot disable motor.")

    def _set_zero_point(self):
        """Sends the 'set zero point' command via serial."""
        if self.serial_handler.ser and self.serial_handler.ser.is_open:
            self.logger.log("Sending 'Set Zero Point' (z) command via serial...")
            self.serial_handler.set_zero_point()
        else:
            self.logger.log("Cannot set zero point: Serial port not connected.")
            QMessageBox.warning(self, "Serial Error", "Serial port not connected. Cannot set zero point.")

    def _emergency_stop(self):
        """Handles the EMERGENCY STOP action: disables motor via serial and sets CAN to safe state."""
        self.logger.log("EMERGENCY STOP triggered!")

        # 1. Disable motor via serial (sends ESC)
        self._disable_motor_serial()

        # 2. Stop any ongoing automated cycle test immediately
        if self.cycle_test_thread and self.cycle_test_thread.isRunning():
            self._stop_cycle_test(force_stop=True) # Force stop without waiting for graceful completion

        # 3. Ensure CAN communication thread sends safe parameters or stops
        if self.can_comm_thread and self.can_comm_thread.isRunning():
            self.can_comm_thread.set_gui_updates_locked(True) # Prevent GUI from further changing params
            # Instruct the CAN thread to continuously send safe parameters (e.g., hold position with zero gains)
            self.can_comm_thread.update_all_params(
                self._last_actual_angle_deg * const.DEG_TO_RAD, # Target current actual position
                0.0,  # Target zero velocity
                0.0,  # Zero Kp gain
                0.0,  # Zero Kd gain
                0.0   # Zero feed-forward torque
            )
            self.logger.log("E-STOP: Instructed CAN thread to send safe parameters (hold position, zero gains).")

            # Additionally, send an immediate one-off safe command directly via can_handler
            # for rapid effect, in case the CAN thread's timer tick is delayed.
            if self.can_handler.is_open:
                self.can_handler.send_command(
                    const.MOTOR_CMD_CAN_ID,
                    self._last_actual_angle_deg * const.DEG_TO_RAD,
                    0.0, 0.0, 0.0, 0.0 # p, v, Kp, Kd, Tff
                )
                self.logger.log("E-STOP: Sent an additional immediate one-off safe command via CAN handler.")

        QMessageBox.critical(self, "EMERGENCY STOP",
                             "Emergency Stop Activated!\nMotor disabled (serial) and CAN commands set to safe state.")

    # --- CAN Control Slot Methods ---
    def _connect_can(self):
        """Handles connecting to the CAN device and starting the CAN communication thread."""
        self.logger.log("Attempting to connect to CAN device...")
        # Using specified timing parameters for 1000k bps: Timing0=0x00, Timing1=0x14
        # AccCode=0x80000000 and Filter=1 are now defaults in can_handler.open()
        if self.can_handler.open(timing0=0x00, timing1=0x14): # AccCode and Filter handled by can_handler defaults
            self.param_panel.set_can_connection_status(True)    # Update ParamPanel UI
            self.motion_panel.set_motion_controls_enabled(True) # Enable MotionPanel controls
            self.logger.log("CAN device opened successfully with Timing0=0x00, Timing1=0x14 (1Mbps).")

            # Stop existing CAN thread if it's running, before starting a new one
            if self.can_comm_thread and self.can_comm_thread.isRunning():
                self.can_comm_thread.stop()
                self.can_comm_thread.wait() # Wait for the thread to finish

            # Get initial Kp, Kd, T_ff values from the ParamPanel
            current_params_tuple = self.param_panel.get_current_parameters()
            self._current_kp, self._current_kd, self._current_t_ff = current_params_tuple

            # Create and start the CAN communication thread
            self.can_comm_thread = CANCommunicationThread(
                can_handler=self.can_handler,
                command_can_id=const.MOTOR_CMD_CAN_ID,
                initial_params=current_params_tuple # Pass (kp, kd, t_ff)
            )
            # Connect signals from the CAN thread
            self.can_comm_thread.dataReceived.connect(self._handle_can_data)
            self.can_comm_thread.canStatusMessage.connect(self.logger.log)
            self.can_comm_thread.finished.connect(self._on_can_thread_finished)

            # Set initial target position for the CAN thread based on the GUI slider's current value
            initial_gui_angle_deg = self.motion_panel.get_target_angle_degrees()
            self._current_target_angle_deg = initial_gui_angle_deg
            # Update the CAN thread's parameters (it respects its internal _gui_updates_locked state)
            self.can_comm_thread.update_target_position(initial_gui_angle_deg * const.DEG_TO_RAD, 0.0) # Target 0 vel
            self.can_comm_thread.update_pd_gains(self._current_kp, self._current_kd)
            self.can_comm_thread.update_feedforward_torque(self._current_t_ff)

            self.can_comm_thread.start() # Start the thread's event loop
            self._plot_start_time = time.time() # Reset plot time origin for new session
            self.display_panel.clear_plots()    # Clear any old plot data

        else: # CAN connection failed
            self.param_panel.set_can_connection_status(False)
            self.motion_panel.set_motion_controls_enabled(False)
            QMessageBox.warning(self, "CAN Connection Error",
                                "Failed to open CAN device. Check ControlCAN.dll, hardware connection, and drivers.")
            self.logger.log("Error: Failed to open CAN device.")

    def _disconnect_can(self):
        """Handles disconnecting from the CAN device and stopping related threads."""
        self.logger.log("Disconnecting from CAN device...")
        # Stop cycle test first if it's running
        if self.cycle_test_thread and self.cycle_test_thread.isRunning():
            self._stop_cycle_test(force_stop=True)

        # Stop the CAN communication thread
        if self.can_comm_thread and self.can_comm_thread.isRunning():
            self.can_comm_thread.stop()
            # The thread's finished signal will trigger _on_can_thread_finished for cleanup.
            # No need to call self.can_handler.close() here if thread handles it.
        else:
            # If thread wasn't running or already stopped, close CAN handler directly
            self.can_handler.close()
            self.param_panel.set_can_connection_status(False)
            self.motion_panel.set_motion_controls_enabled(False)
            self.logger.log("CAN device closed (thread was not active).")


    def _on_can_thread_finished(self):
        """Slot called when the CANCommunicationThread finishes its execution."""
        self.logger.log("CAN communication thread has finished.")
        self.can_handler.close() # Ensure CAN device is closed
        # Update UI states
        self.param_panel.set_can_connection_status(False)
        self.motion_panel.set_motion_controls_enabled(False)
        self.can_comm_thread = None # Clear the reference


    def _update_pd_parameters(self, kp, kd, t_ff):
        """Handles updates to Kp, Kd, T_ff from the ParamPanel."""
        self._current_kp = kp
        self._current_kd = kd
        self._current_t_ff = t_ff # t_ff is now part of the parameters signal

        if self.can_comm_thread and self.can_comm_thread.isRunning():
            self.can_comm_thread.update_pd_gains(self._current_kp, self._current_kd)
            self.can_comm_thread.update_feedforward_torque(self._current_t_ff)
        self.logger.log(f"PD & Tff parameters updated: Kp={kp:.2f}, Kd={kd:.2f}, T_ff={t_ff:.2f}")

    def _update_target_angle(self, angle_degrees):
        """Handles target angle changes from the MotionPanel's slider."""
        self._current_target_angle_deg = angle_degrees
        if self.can_comm_thread and self.can_comm_thread.isRunning():
            # Send target position (radians) and zero target velocity for position hold
            self.can_comm_thread.update_target_position(angle_degrees * const.DEG_TO_RAD, 0.0)
        # Logging this can be very verbose if slider is moved rapidly, so usually commented out.
        # self.logger.log(f"Target angle set to: {angle_degrees:.1f}°")

    def _handle_can_data(self, motor_id, p_rad, v_rad_s, t_nm):
        """
        Processes data received from the CANCommunicationThread.
        Updates numerical displays, joint diagram, and real-time plot.
        Args:
            motor_id (int): ID of the motor sending the data (from message payload).
            p_rad (float): Actual position in radians.
            v_rad_s (float): Actual velocity in rad/s.
            t_nm (float): Actual torque in Nm.
        """
        # Filter data for the motor we are interested in (e.g., MOTOR_CMD_CAN_ID).
        # This is important if multiple motors could be on the bus.
        if motor_id != const.MOTOR_CMD_CAN_ID: # Or a specific reply ID if different
            # self.logger.log(f"Received data from unexpected motor ID: {motor_id}. Ignoring.")
            return

        # Convert radians to degrees for GUI display
        p_deg = p_rad * const.RAD_TO_DEG
        v_deg_s = v_rad_s * const.RAD_TO_DEG
        self._last_actual_angle_deg = p_deg # Store for E-Stop and plot

        # Update GUI elements
        self.display_panel.update_numerical_display(p_deg, v_deg_s, t_nm)
        self.display_panel.update_joint_diagram(p_deg)

        current_plot_time = time.time() - self._plot_start_time
        self.display_panel.update_plot(current_plot_time, self._current_target_angle_deg, p_deg)

        # Provide actual position to cycle test thread if it's running (for potential feedback/logging)
        if self.cycle_test_thread and self.cycle_test_thread.isRunning():
            self.cycle_test_thread.update_actual_position(p_rad)


    # --- Cycle Test Slot Methods ---
    def _start_cycle_test(self, cycles, acceleration_rad_s2, max_velocity_rad_s):
        """Starts the automated cycle test."""
        if not (self.can_comm_thread and self.can_comm_thread.isRunning()):
            QMessageBox.warning(self, "CAN Error", "CAN is not connected. Cannot start cycle test.")
            self.logger.log("Cycle test start failed: CAN not connected.")
            return

        if self.cycle_test_thread and self.cycle_test_thread.isRunning():
            QMessageBox.warning(self, "Cycle Test", "Cycle test is already running.")
            self.logger.log("Cycle test start failed: Already running.")
            return

        self.logger.log(f"Starting cycle test: {cycles} cycles, Accel={acceleration_rad_s2} rad/s^2, MaxVel={max_velocity_rad_s} rad/s.")
        self.motion_panel.set_cycle_test_running(True) # Update MotionPanel's UI state
        if self.can_comm_thread:
            self.can_comm_thread.set_gui_updates_locked(True) # CycleTestThread will now control CAN params

        # Use Kp, Kd from ParamPanel for the cycle test. T_ff for cycle test is usually 0.
        current_kp, current_kd, _ = self.param_panel.get_current_parameters()

        self.cycle_test_thread = CycleTestThread(
            num_cycles=cycles,
            accel=acceleration_rad_s2,
            max_vel=max_velocity_rad_s,
            initial_kp=current_kp,
            initial_kd=current_kd
        )
        # Connect signals from CycleTestThread
        self.cycle_test_thread.newCANCommandParams.connect(self.can_comm_thread.update_all_params)
        self.cycle_test_thread.cycleStatusUpdate.connect(self.logger.log)
        self.cycle_test_thread.cycleFinished.connect(self._on_cycle_test_finished)
        self.cycle_test_thread.start() # Start the thread

    def _stop_cycle_test(self, force_stop=False):
        """Stops the currently running automated cycle test."""
        if self.cycle_test_thread and self.cycle_test_thread.isRunning():
            self.logger.log("Stopping cycle test...")
            self.cycle_test_thread.stop() # Signal the thread to stop gracefully
            if force_stop:
                # If forced (e.g., E-Stop), trigger cleanup immediately without waiting for thread.
                # The thread might still be running but will terminate soon.
                self._on_cycle_test_finished()
        else:
            # Avoid logging "No cycle test running" if part of a forced E-Stop cascade.
            if not force_stop:
                self.logger.log("No cycle test currently running to stop.")


    def _on_cycle_test_finished(self):
        """Slot called when the CycleTestThread finishes or is stopped."""
        self.logger.log("Cycle test thread has finished its execution.")
        if self.can_comm_thread and self.can_comm_thread.isRunning():
            self.can_comm_thread.set_gui_updates_locked(False) # Release lock for GUI control

            # Restore CAN thread's parameters to current GUI control values
            current_gui_angle_deg = self.motion_panel.get_target_angle_degrees()
            self._current_target_angle_deg = current_gui_angle_deg

            kp, kd, t_ff = self.param_panel.get_current_parameters()
            self._current_kp = kp
            self._current_kd = kd
            self._current_t_ff = t_ff

            # Update CAN thread to reflect current GUI state (e.g., slider position, PD gains)
            self.can_comm_thread.update_all_params(
                p_des_rad = current_gui_angle_deg * const.DEG_TO_RAD,
                v_des_rad_s = 0.0, # Default to zero velocity target after cycle test
                kp = kp,
                kd = kd,
                t_ff_nm = t_ff
            )
            self.logger.log(f"Restored CAN thread params to GUI values: P={current_gui_angle_deg:.1f}, Kp={kp:.1f}, Kd={kd:.1f}, Tff={t_ff:.1f}")

        self.motion_panel.set_cycle_test_running(False) # Update MotionPanel's UI state

        # Set motion panel slider to the end point of the cycle (e.g., 0 degrees) for smooth transition.
        self.motion_panel.set_target_angle_degrees(const.CYCLE_POS_A_RAD * const.RAD_TO_DEG)

        self.cycle_test_thread = None # Clear the reference


    # --- Window Management Methods ---
    def closeEvent(self, event):
        """Handles the window close event to ensure graceful shutdown of threads and resources."""
        self.logger.log("Close event triggered. Cleaning up application resources...")

        # Stop active threads first
        if self.cycle_test_thread and self.cycle_test_thread.isRunning():
            self.logger.log("Stopping cycle test thread before closing...")
            self.cycle_test_thread.stop()
            self.cycle_test_thread.wait(1000) # Wait up to 1 sec for thread to finish

        if self.can_comm_thread and self.can_comm_thread.isRunning():
            self.logger.log("Stopping CAN communication thread before closing...")
            self.can_comm_thread.stop()
            self.can_comm_thread.wait(1000) # Wait up to 1 sec; it should close CAN device

        # Ensure CAN device is closed if thread didn't handle it or wasn't running
        if self.can_handler.is_open:
             self.logger.log("Ensuring CAN device is closed...")
             self.can_handler.close()

        # Ensure serial port is closed
        if self.serial_handler.ser and self.serial_handler.ser.is_open:
            self.logger.log("Ensuring serial port is closed...")
            self.serial_handler.disconnect()

        self._save_settings() # Save window geometry and other settings
        self.logger.log("Application cleanup finished. Exiting.")
        super().closeEvent(event) # Proceed with closing

    def _load_settings(self):
        """Loads window geometry and other saved settings using QSettings."""
        self.logger.log("Loading window settings (geometry, splitter state)...")
        # Restore window geometry (size and position)
        geometry = self._settings.value("MainWindow/geometry")
        if geometry:
            self.restoreGeometry(geometry)

        # Restore splitter state
        splitter_state = self._settings.value("MainWindow/splitterState")
        if splitter_state:
            # Find the QSplitter child of the central widget (assuming there's only one main splitter)
            splitter = self.central_widget.findChild(QSplitter)
            if splitter:
                splitter.restoreState(splitter_state)

        # Example: Load last used COM port (optional feature)
        # last_com_port = self._settings.value("Serial/lastCOMPort", "")
        # if last_com_port:
        #     idx = self.control_panel.port_combo.findText(last_com_port)
        #     if idx >= 0:
        #         self.control_panel.port_combo.setCurrentIndex(idx)

    def _save_settings(self):
        """Saves current window geometry and other settings using QSettings."""
        self.logger.log("Saving window settings...")
        self._settings.setValue("MainWindow/geometry", self.saveGeometry())

        splitter = self.central_widget.findChild(QSplitter)
        if splitter:
            self._settings.setValue("MainWindow/splitterState", splitter.saveState())

        # Example: Save last used COM port (optional feature)
        # current_com_port = self.control_panel.port_combo.currentText()
        # if current_com_port: # Only save if a port is selected/available
        #     self._settings.setValue("Serial/lastCOMPort", current_com_port)


if __name__ == '__main__':
    # This block is for testing MainWindow directly.
    # It includes workarounds for relative imports if __package__ is not set (direct execution).
    # Best practice is to run the application via `python -m test.main` from the project root.
    if __package__ is None or __package__ == '':
        # This adjustment helps if running `python main_window.py` directly from inside `test/gui/`
        # by adding the `test` directory's parent (project root) to sys.path,
        # allowing `from test.gui...` or `from test.utils...` to work.
        import os
        script_dir = os.path.dirname(os.path.abspath(__file__)) # .../test/gui
        test_dir = os.path.dirname(script_dir) # .../test
        project_root = os.path.dirname(test_dir) # .../
        if project_root not in sys.path:
            sys.path.insert(0, project_root)

        # After path adjustment, re-attempt imports as if 'test' is a package
        # This requires changing the module's own relative imports to be absolute from 'test'
        # For direct execution test, it's simpler if the imports at the top of the file
        # are already structured assuming 'test' is discoverable (e.g. `from test.gui.widgets...`)
        # or if this __main__ block re-imports them after path setup.
        # The current imports like `from .widgets.control_panel import ControlPanel`
        # are for when `main_window.py` is treated as part of the `test.gui` package.
        # To run this standalone for testing, you might need to change them or use:
        try:
            from test.gui.widgets.control_panel import ControlPanel
            from test.gui.widgets.motion_panel import MotionPanel
            from test.gui.widgets.param_panel import ParamPanel
            from test.gui.widgets.display_panel import DisplayPanel
            from test.serial_communication.serial_handler import SerialHandler
            from test.can_communication.can_handler import CANHandler
            from test.utils import constants as const # Re-import as absolute from test
            from test.utils.logger import GUILogger
        except ImportError: # Fallback if the path adjustment didn't make 'test' a root for imports
            print("Warning: Running main_window.py directly. Re-check imports or run via test.main.")
            # The original relative imports might work if Python's CWD is test/gui or similar.

    app = QApplication(sys.argv)
    # Optional: Apply a global style for a more modern look (e.g., "Fusion")
    # app.setStyle("Fusion")

    main_window_instance = MainWindow()
    main_window_instance.show() # Display the main window

    sys.exit(app.exec()) # Start the Qt application event loop
