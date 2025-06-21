from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QComboBox, QLabel, QGroupBox, QSizePolicy)
from PyQt6.QtCore import pyqtSignal

class ControlPanel(QWidget):
    """
    ControlPanel widget for handling serial communication setup and basic motor commands.
    This panel includes:
    - COM port selection with auto-refresh.
    - Connect/Disconnect buttons for the serial port.
    - Buttons for motor operations: Enable, Disable, Set Zero Point, Emergency Stop.

    Signals are emitted for these actions to be handled by the main application logic.
    """

    # --- Signals ---
    # Emitted when the "Connect Serial" button is clicked, carrying the selected port name.
    connectSerialClicked = pyqtSignal(str)
    # Emitted when the "Disconnect Serial" button is clicked.
    disconnectSerialClicked = pyqtSignal()
    # Emitted when the "Enable Motor" button is clicked.
    enableMotorClicked = pyqtSignal()
    # Emitted when the "Disable Motor" button is clicked.
    disableMotorClicked = pyqtSignal()
    # Emitted when the "Set Zero Point" button is clicked.
    setZeroPointClicked = pyqtSignal()
    # Emitted when the "EMERGENCY STOP" button is clicked.
    emergencyStopClicked = pyqtSignal() # Functionally similar to disableMotor, but for critical stop.
    # Emitted when the "Refresh" button for COM ports is clicked.
    refreshPortsClicked = pyqtSignal()

    def __init__(self, parent=None):
        """
        Initializes the ControlPanel widget.
        Args:
            parent (QWidget, optional): The parent widget. Defaults to None.
        """
        super().__init__(parent)
        self.setObjectName("ControlPanel") # For styling or identification
        self._init_ui() # Initialize user interface elements

    def _init_ui(self):
        """Sets up the layout and widgets for the control panel."""
        main_layout = QVBoxLayout(self) # Main vertical layout for the panel

        # --- Serial Connection Group ---
        serial_group = QGroupBox("Serial Connection") # GroupBox for serial port elements
        serial_layout = QVBoxLayout() # Layout for elements within the serial group

        # COM Port selection row (label, combobox, refresh button)
        port_layout = QHBoxLayout()
        port_label = QLabel("COM Port:")
        self.port_combo = QComboBox()
        self.port_combo.setToolTip("Select the COM port for the motor controller (e.g., Silicon Labs CP210x).")
        self.port_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed) # Combobox expands

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.setToolTip("Refresh list of available COM ports.")
        self.refresh_button.clicked.connect(self.refreshPortsClicked) # Emit signal on click

        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_combo)
        port_layout.addWidget(self.refresh_button)
        serial_layout.addLayout(port_layout)

        # Connect/Disconnect buttons for serial port
        self.connect_button = QPushButton("Connect Serial")
        self.connect_button.setToolTip("Connect to the selected serial port.")
        self.connect_button.clicked.connect(self._on_connect_clicked) # Internal slot then emits signal

        self.disconnect_button = QPushButton("Disconnect Serial")
        self.disconnect_button.setToolTip("Disconnect from the current serial port.")
        self.disconnect_button.clicked.connect(self.disconnectSerialClicked) # Emit signal directly
        self.disconnect_button.setEnabled(False) # Initially disabled until connected

        serial_layout.addWidget(self.connect_button)
        serial_layout.addWidget(self.disconnect_button)
        serial_group.setLayout(serial_layout)
        main_layout.addWidget(serial_group)

        # --- Motor Control Group (Serial Commands) ---
        motor_control_group = QGroupBox("Motor Commands (Serial)")
        motor_control_layout = QVBoxLayout()

        self.enable_button = QPushButton("Enable Motor")
        self.enable_button.setToolTip("Send 'm' command via serial to enable the motor.")
        self.enable_button.clicked.connect(self.enableMotorClicked)

        self.disable_button = QPushButton("Disable Motor")
        self.disable_button.setToolTip("Send 'ESC' command via serial to disable the motor.")
        self.disable_button.clicked.connect(self.disableMotorClicked)

        self.set_zero_button = QPushButton("Set Zero Point")
        self.set_zero_button.setToolTip("Send 'z' command via serial to set current motor position as zero.")
        self.set_zero_button.clicked.connect(self.setZeroPointClicked)

        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setToolTip("Immediately disable motor (sends 'ESC' command). Critical safety feature.")
        self.emergency_stop_button.setStyleSheet("background-color: red; color: white; font-weight: bold;") # Style for emphasis
        self.emergency_stop_button.clicked.connect(self.emergencyStopClicked)

        motor_control_layout.addWidget(self.enable_button)
        motor_control_layout.addWidget(self.disable_button)
        motor_control_layout.addWidget(self.set_zero_button)
        motor_control_layout.addSpacing(10) # Add some space before E-STOP
        motor_control_layout.addWidget(self.emergency_stop_button)

        motor_control_group.setLayout(motor_control_layout)
        main_layout.addWidget(motor_control_group)

        # Motor control buttons are initially disabled; enabled upon successful serial connection.
        self.set_motor_commands_enabled(False)

        main_layout.addStretch() # Pushes all content to the top if space is available

    def _on_connect_clicked(self):
        """Internal slot for the connect button. Emits connectSerialClicked signal with port name."""
        port = self.port_combo.currentText()
        if port: # Ensure a port is selected
            self.connectSerialClicked.emit(port)
        # The main window will handle the actual connection and then call set_connection_status.

    def populate_ports(self, port_list):
        """
        Populates the COM port QComboBox with a list of available port names.
        Called by the main window after refreshing ports.

        Args:
            port_list (list[str]): List of COM port names.
        """
        current_port_selection = self.port_combo.currentText() # Preserve current selection if still valid
        self.port_combo.clear()
        self.port_combo.addItems(port_list)

        if current_port_selection in port_list: # Restore previous selection if it's in the new list
            self.port_combo.setCurrentText(current_port_selection)
        elif port_list: # Otherwise, select the first port in the new list
            self.port_combo.setCurrentIndex(0)

    def set_connection_status(self, is_connected):
        """
        Updates the UI elements (buttons, combobox) based on the serial connection status.
        Called by the main window.

        Args:
            is_connected (bool): True if connected, False otherwise.
        """
        self.connect_button.setEnabled(not is_connected)
        self.disconnect_button.setEnabled(is_connected)
        self.port_combo.setEnabled(not is_connected) # Disable port selection when connected
        self.refresh_button.setEnabled(not is_connected) # Disable refresh when connected

        # Motor command buttons are enabled only if serial connection is active.
        self.set_motor_commands_enabled(is_connected)

    def set_motor_commands_enabled(self, enabled):
        """
        Enables or disables all motor command buttons.

        Args:
            enabled (bool): True to enable, False to disable.
        """
        self.enable_button.setEnabled(enabled)
        self.disable_button.setEnabled(enabled)
        self.set_zero_button.setEnabled(enabled)
        # Emergency stop button relies on serial communication, so its state is tied to connection.
        self.emergency_stop_button.setEnabled(enabled)


if __name__ == '__main__':
    # This block allows testing the ControlPanel widget independently.
    import sys
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    app = QApplication(sys.argv)

    # Create and show the ControlPanel instance for testing
    test_control_panel = ControlPanel()

    # --- Test Data & Slot Functions ---
    def simulate_handle_connect(port_name):
        print(f"TEST SLOT: ConnectSerialClicked for port '{port_name}'")
        # Simulate a successful connection for UI update
        if "COM4" in port_name: # Assume COM4 is our target device for test
            print("TEST: Simulating successful connection to COM4.")
            test_control_panel.set_connection_status(True)
        else:
            print(f"TEST: Simulating failed connection to {port_name}.")
            test_control_panel.set_connection_status(False)

    def simulate_handle_disconnect():
        print("TEST SLOT: DisconnectSerialClicked")
        test_control_panel.set_connection_status(False)

    def simulate_handle_refresh_ports():
        print("TEST SLOT: RefreshPortsClicked")
        # Simulate finding some ports
        simulated_ports = ["COM1", "COM3 (Other)", "COM4 (Silicon Labs CP210x)", "COM7"]
        print(f"TEST: Populating ports with: {simulated_ports}")
        test_control_panel.populate_ports(simulated_ports)

    # Connect signals to test slots
    test_control_panel.connectSerialClicked.connect(simulate_handle_connect)
    test_control_panel.disconnectSerialClicked.connect(simulate_handle_disconnect)
    test_control_panel.refreshPortsClicked.connect(simulate_handle_refresh_ports)

    # Connect motor command signals to simple print lambdas for verification
    test_control_panel.enableMotorClicked.connect(lambda: print("TEST SLOT: EnableMotorClicked"))
    test_control_panel.disableMotorClicked.connect(lambda: print("TEST SLOT: DisableMotorClicked"))
    test_control_panel.setZeroPointClicked.connect(lambda: print("TEST SLOT: SetZeroPointClicked"))
    test_control_panel.emergencyStopClicked.connect(lambda: print("TEST SLOT: EmergencyStopClicked"))

    test_control_panel.setWindowTitle("Control Panel - Standalone Test")
    test_control_panel.show()

    # --- Automated Test Steps ---
    print("\n--- Control Panel Test Steps ---")
    # 1. Initial state: Motor commands should be disabled.
    print(f"Initial: Enable button enabled? {test_control_panel.enable_button.isEnabled()} (Expected: False)")
    assert not test_control_panel.enable_button.isEnabled(), "Initial state: Enable button should be disabled."

    # 2. Simulate refreshing ports
    print("\nStep 2: Simulating port refresh...")
    test_control_panel.refresh_button.click() # Triggers refreshPortsClicked -> simulate_handle_refresh_ports

    # 3. Simulate connecting to a "good" port (e.g., COM4)
    print("\nStep 3: Simulating connection to 'COM4 (Silicon Labs CP210x)'...")
    # Find and set the "good" port in combo if available from refresh
    good_port_text = "COM4 (Silicon Labs CP210x)"
    index = test_control_panel.port_combo.findText(good_port_text)
    if index != -1:
        test_control_panel.port_combo.setCurrentIndex(index)
        test_control_panel.connect_button.click() # Triggers connectSerialClicked -> simulate_handle_connect
        print(f"After connect attempt: Enable button enabled? {test_control_panel.enable_button.isEnabled()} (Expected: True)")
        assert test_control_panel.enable_button.isEnabled(), "After connect: Enable button should be enabled."
        assert test_control_panel.disconnect_button.isEnabled(), "After connect: Disconnect button should be enabled."
        assert not test_control_panel.connect_button.isEnabled(), "After connect: Connect button should be disabled."
    else:
        print(f"'{good_port_text}' not found in ComboBox after refresh, skipping connect test part.")

    # 4. Simulate disconnecting
    if test_control_panel.disconnect_button.isEnabled(): # Only if connection was simulated successfully
        print("\nStep 4: Simulating disconnection...")
        test_control_panel.disconnect_button.click() # Triggers disconnectSerialClicked -> simulate_handle_disconnect
        print(f"After disconnect: Enable button enabled? {test_control_panel.enable_button.isEnabled()} (Expected: False)")
        assert not test_control_panel.enable_button.isEnabled(), "After disconnect: Enable button should be disabled."
        assert not test_control_panel.disconnect_button.isEnabled(), "After disconnect: Disconnect button should be disabled."
        assert test_control_panel.connect_button.isEnabled(), "After disconnect: Connect button should be enabled."

    print("\n--- Control Panel Test End ---")
    sys.exit(app.exec())
