from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDial,
                             QDoubleSpinBox, QGroupBox, QGridLayout, QPushButton)
from PyQt6.QtCore import Qt, pyqtSignal
from test.utils import constants as const # Using absolute import from 'test'

class ParamPanel(QWidget):
    """
    ParamPanel widget for managing CAN connection and adjusting motor control parameters.
    This panel provides:
    - Buttons to connect/disconnect the CAN interface.
    - Dials and Spinboxes for real-time tuning of Kp (proportional gain),
      Kd (derivative gain), and T_ff (feed-forward torque).

    Changes to parameters are emitted via a signal for the main application to handle.
    """

    # --- Signals ---
    # Emitted when any of the PD or T_ff parameters are changed by the user.
    # Carries the new Kp, Kd, and T_ff values.
    parametersChanged = pyqtSignal(float, float, float)
    # Emitted when the "Connect CAN" button is clicked.
    connectCANClicked = pyqtSignal()
    # Emitted when the "Disconnect CAN" button is clicked.
    disconnectCANClicked = pyqtSignal()


    def __init__(self, parent=None):
        """
        Initializes the ParamPanel widget.
        Args:
            parent (QWidget, optional): The parent widget. Defaults to None.
        """
        super().__init__(parent)
        self.setObjectName("ParamPanel") # For styling or identification
        self._init_ui() # Setup the user interface elements

    def _init_ui(self):
        """Sets up the layout and widgets for the parameter panel."""
        main_layout = QVBoxLayout(self) # Main vertical layout for the panel

        # --- CAN Connection Group ---
        can_connection_group = QGroupBox("CAN Connection")
        can_connection_layout = QVBoxLayout()

        self.connect_can_button = QPushButton("Connect CAN")
        self.connect_can_button.setToolTip("Initialize and start CAN communication for motor control.")
        self.connect_can_button.clicked.connect(self.connectCANClicked) # Emit signal

        self.disconnect_can_button = QPushButton("Disconnect CAN")
        self.disconnect_can_button.setToolTip("Stop and release CAN communication.")
        self.disconnect_can_button.clicked.connect(self.disconnectCANClicked) # Emit signal
        self.disconnect_can_button.setEnabled(False) # Disabled until CAN is connected

        can_connection_layout.addWidget(self.connect_can_button)
        can_connection_layout.addWidget(self.disconnect_can_button)
        can_connection_group.setLayout(can_connection_layout)
        main_layout.addWidget(can_connection_group)

        # --- Parameter Adjustment Group (using QGridLayout for Dials and Spinboxes) ---
        param_group = QGroupBox("Parameter Adjustment (CAN)")
        param_grid_layout = QGridLayout() # Use a grid for better alignment

        # Kp (Proportional Gain) Widgets
        kp_label = QLabel("Kp (Nm/rad):")
        self.kp_dial = QDial()
        # Dials work with integers, so scale float values (e.g., by 100 for 2 decimal places)
        self.kp_dial.setRange(int(const.PARAM_KP_MIN_DISPLAY * 100), int(const.PARAM_KP_MAX_DISPLAY * 100))
        self.kp_dial.setValue(int(const.DEFAULT_KP * 100)) # Set initial default value
        self.kp_dial.setNotchesVisible(True) # Show notches for better visual guidance
        self.kp_dial.setWrapping(False) # Kp is typically non-negative, no wrapping
        self.kp_dial.valueChanged.connect(self._on_kp_dial_changed) # Connect dial change to slot

        self.kp_spinbox = QDoubleSpinBox() # Spinbox for precise Kp input
        self.kp_spinbox.setRange(const.PARAM_KP_MIN_DISPLAY, const.PARAM_KP_MAX_DISPLAY)
        self.kp_spinbox.setSingleStep(0.1) # Step size for spinbox arrows
        self.kp_spinbox.setValue(const.DEFAULT_KP)
        self.kp_spinbox.setDecimals(2) # Display two decimal places
        self.kp_spinbox.valueChanged.connect(self._on_kp_spinbox_changed) # Connect spinbox change to slot

        param_grid_layout.addWidget(kp_label, 0, 0) # Row 0, Col 0
        param_grid_layout.addWidget(self.kp_dial, 0, 1) # Row 0, Col 1
        param_grid_layout.addWidget(self.kp_spinbox, 0, 2) # Row 0, Col 2

        # Kd (Derivative Gain) Widgets
        kd_label = QLabel("Kd (Nm/(rad/s)):")
        self.kd_dial = QDial()
        self.kd_dial.setRange(int(const.PARAM_KD_MIN_DISPLAY * 100), int(const.PARAM_KD_MAX_DISPLAY * 100))
        self.kd_dial.setValue(int(const.DEFAULT_KD * 100))
        self.kd_dial.setNotchesVisible(True)
        self.kd_dial.setWrapping(False)
        self.kd_dial.valueChanged.connect(self._on_kd_dial_changed)

        self.kd_spinbox = QDoubleSpinBox()
        self.kd_spinbox.setRange(const.PARAM_KD_MIN_DISPLAY, const.PARAM_KD_MAX_DISPLAY)
        self.kd_spinbox.setSingleStep(0.01)
        self.kd_spinbox.setValue(const.DEFAULT_KD)
        self.kd_spinbox.setDecimals(2)
        self.kd_spinbox.valueChanged.connect(self._on_kd_spinbox_changed)

        param_grid_layout.addWidget(kd_label, 1, 0) # Row 1, Col 0
        param_grid_layout.addWidget(self.kd_dial, 1, 1) # Row 1, Col 1
        param_grid_layout.addWidget(self.kd_spinbox, 1, 2) # Row 1, Col 2

        # T_ff (Feed-forward Torque) Widgets
        t_ff_label = QLabel("T_ff (Nm):")
        self.t_ff_dial = QDial()
        self.t_ff_dial.setRange(int(const.PARAM_T_FF_MIN_DISPLAY * 100), int(const.PARAM_T_FF_MAX_DISPLAY * 100))
        self.t_ff_dial.setValue(int(const.DEFAULT_T_FF * 100))
        self.t_ff_dial.setNotchesVisible(True)
        self.t_ff_dial.setWrapping(False) # FF torque can be negative, but dial range handles it.
        self.t_ff_dial.valueChanged.connect(self._on_t_ff_dial_changed)

        self.t_ff_spinbox = QDoubleSpinBox()
        self.t_ff_spinbox.setRange(const.PARAM_T_FF_MIN_DISPLAY, const.PARAM_T_FF_MAX_DISPLAY)
        self.t_ff_spinbox.setSingleStep(0.05)
        self.t_ff_spinbox.setValue(const.DEFAULT_T_FF)
        self.t_ff_spinbox.setDecimals(2)
        self.t_ff_spinbox.valueChanged.connect(self._on_t_ff_spinbox_changed)

        param_grid_layout.addWidget(t_ff_label, 2, 0) # Row 2, Col 0
        param_grid_layout.addWidget(self.t_ff_dial, 2, 1) # Row 2, Col 1
        param_grid_layout.addWidget(self.t_ff_spinbox, 2, 2) # Row 2, Col 2

        param_group.setLayout(param_grid_layout)
        main_layout.addWidget(param_group)

        # Parameter adjustment widgets are initially disabled; enabled upon successful CAN connection.
        self.set_params_enabled(False)

        main_layout.addStretch() # Push content to the top
        self.setLayout(main_layout)

    # --- Slots for Parameter Changes ---
    # These slots synchronize the Dial and SpinBox for each parameter and emit the parametersChanged signal.

    def _on_kp_dial_changed(self, value):
        """Handles Kp QDial value change: updates Kp QDoubleSpinBox and emits parametersChanged."""
        # Unscale dial value (integer) to float for spinbox
        self.kp_spinbox.setValue(float(value) / 100.0)
        self._emit_parameters_changed() # Emit signal with all current parameters
    def _on_kp_spinbox_changed(self, value):
        """Handles Kp QDoubleSpinBox value change: updates Kp QDial."""
        # Scale spinbox value (float) to integer for dial
        self.kp_dial.setValue(int(value * 100))
        # Note: parametersChanged is not emitted here to avoid double emission,
        # as the dial change (triggered by this) will emit it.
        # If direct spinbox changes should also immediately emit, add it here and manage potential recursion.
        # For simplicity, assuming dial change is the primary trigger for emission after sync.

    def _on_kd_dial_changed(self, value):
        """Handles Kd QDial value change: updates Kd QDoubleSpinBox and emits parametersChanged."""
        self.kd_spinbox.setValue(float(value) / 100.0)
        self._emit_parameters_changed()
    def _on_kd_spinbox_changed(self, value):
        """Handles Kd QDoubleSpinBox value change: updates Kd QDial."""
        self.kd_dial.setValue(int(value * 100))

    def _on_t_ff_dial_changed(self, value):
        """Handles T_ff QDial value change: updates T_ff QDoubleSpinBox and emits parametersChanged."""
        self.t_ff_spinbox.setValue(float(value) / 100.0)
        self._emit_parameters_changed()
    def _on_t_ff_spinbox_changed(self, value):
        """Handles T_ff QDoubleSpinBox value change: updates T_ff QDial."""
        self.t_ff_dial.setValue(int(value * 100))

    def _emit_parameters_changed(self):
        """Helper method to gather current Kp, Kd, T_ff values and emit the signal."""
        kp = self.kp_spinbox.value()
        kd = self.kd_spinbox.value()
        t_ff = self.t_ff_spinbox.value()
        self.parametersChanged.emit(kp, kd, t_ff)

    def get_current_parameters(self):
        """
        Returns the current values of Kp, Kd, and T_ff from the spinboxes.
        Returns:
            tuple: (float, float, float) for Kp, Kd, T_ff.
        """
        return self.kp_spinbox.value(), self.kd_spinbox.value(), self.t_ff_spinbox.value()

    # --- UI State Management ---
    def set_can_connection_status(self, is_connected):
        """
        Updates UI elements based on CAN connection status. Called by the main window.
        Args:
            is_connected (bool): True if CAN is connected, False otherwise.
        """
        self.connect_can_button.setEnabled(not is_connected)
        self.disconnect_can_button.setEnabled(is_connected)
        self.set_params_enabled(is_connected) # Enable/disable parameter tuning widgets

    def set_params_enabled(self, enabled):
        """
        Enables or disables all parameter adjustment widgets (dials and spinboxes).
        Args:
            enabled (bool): True to enable, False to disable.
        """
        self.kp_dial.setEnabled(enabled)
        self.kp_spinbox.setEnabled(enabled)
        self.kd_dial.setEnabled(enabled)
        self.kd_spinbox.setEnabled(enabled)
        self.t_ff_dial.setEnabled(enabled)
        self.t_ff_spinbox.setEnabled(enabled)

if __name__ == '__main__':
    # This block allows testing the ParamPanel widget independently.
    import sys
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    # Create and show the ParamPanel instance for testing
    test_param_panel = ParamPanel()

    # --- Test Slot Functions ---
    def simulate_handle_params_changed(kp, kd, t_ff):
        print(f"TEST SLOT: ParametersChanged: Kp={kp:.2f}, Kd={kd:.2f}, T_ff={t_ff:.2f}")

    def simulate_handle_connect_can():
        print("TEST SLOT: ConnectCANClicked")
        # Simulate successful CAN connection for UI update
        test_param_panel.set_can_connection_status(True)

    def simulate_handle_disconnect_can():
        print("TEST SLOT: DisconnectCANClicked")
        # Simulate CAN disconnection for UI update
        test_param_panel.set_can_connection_status(False)

    # Connect signals to test slots
    test_param_panel.parametersChanged.connect(simulate_handle_params_changed)
    test_param_panel.connectCANClicked.connect(simulate_handle_connect_can)
    test_param_panel.disconnectCANClicked.connect(simulate_handle_disconnect_can)

    test_param_panel.setWindowTitle("Parameter Panel - Standalone Test")
    test_param_panel.show()

    # --- Automated Test Steps ---
    print("\n--- Parameter Panel Test Steps ---")
    # 1. Initial state: Parameter controls should be disabled, Connect CAN should be enabled.
    print(f"Initial: Kp dial enabled? {test_param_panel.kp_dial.isEnabled()} (Expected: False)")
    assert not test_param_panel.kp_dial.isEnabled(), "Initial state: Kp dial should be disabled."
    print(f"Initial: Connect CAN enabled? {test_param_panel.connect_can_button.isEnabled()} (Expected: True)")
    assert test_param_panel.connect_can_button.isEnabled(), "Initial state: Connect CAN button should be enabled."
    print(f"Initial: Disconnect CAN enabled? {test_param_panel.disconnect_can_button.isEnabled()} (Expected: False)")
    assert not test_param_panel.disconnect_can_button.isEnabled(), "Initial state: Disconnect CAN button should be disabled."

    # 2. Simulate CAN connection
    print("\nStep 2: Simulating CAN connection...")
    test_param_panel.connect_can_button.click() # Triggers connectCANClicked -> simulate_handle_connect_can

    print(f"After CAN connect: Kp dial enabled? {test_param_panel.kp_dial.isEnabled()} (Expected: True)")
    assert test_param_panel.kp_dial.isEnabled(), "After CAN connect: Kp dial should be enabled."
    print(f"After CAN connect: Connect CAN enabled? {test_param_panel.connect_can_button.isEnabled()} (Expected: False)")
    assert not test_param_panel.connect_can_button.isEnabled(), "After CAN connect: Connect CAN button should be disabled."
    print(f"After CAN connect: Disconnect CAN enabled? {test_param_panel.disconnect_can_button.isEnabled()} (Expected: True)")
    assert test_param_panel.disconnect_can_button.isEnabled(), "After CAN connect: Disconnect CAN button should be enabled."

    # 3. Simulate changing Kp via dial - this should trigger parametersChanged signal
    print("\nStep 3: Simulating Kp dial change to 25.50...")
    test_param_panel.kp_dial.setValue(int(25.5 * 100)) # Set Kp to 25.50
    current_kp, _, _ = test_param_panel.get_current_parameters()
    print(f"Kp value from get_current_parameters(): {current_kp:.2f} (Expected: 25.50)")
    assert abs(current_kp - 25.5) < 0.001, "Kp dial change not reflected correctly."

    # 4. Simulate changing Kd via spinbox - this should also lead to parametersChanged (via dial update)
    print("\nStep 4: Simulating Kd spinbox change to 0.75...")
    test_param_panel.kd_spinbox.setValue(0.75) # Set Kd to 0.75
    _, current_kd, _ = test_param_panel.get_current_parameters()
    print(f"Kd value from get_current_parameters(): {current_kd:.2f} (Expected: 0.75)")
    assert abs(current_kd - 0.75) < 0.001, "Kd spinbox change not reflected correctly."
    # Verify dial also updated
    print(f"Kd dial value: {test_param_panel.kd_dial.value()} (Expected: {int(0.75*100)})")
    assert test_param_panel.kd_dial.value() == int(0.75 * 100), "Kd dial not synced with spinbox."

    # 5. Simulate CAN disconnection
    print("\nStep 5: Simulating CAN disconnection...")
    test_param_panel.disconnect_can_button.click() # Triggers disconnectCANClicked -> simulate_handle_disconnect_can
    print(f"After CAN disconnect: Kp dial enabled? {test_param_panel.kp_dial.isEnabled()} (Expected: False)")
    assert not test_param_panel.kp_dial.isEnabled(), "After CAN disconnect: Kp dial should be disabled."
    print(f"After CAN disconnect: Connect CAN enabled? {test_param_panel.connect_can_button.isEnabled()} (Expected: True)")
    assert test_param_panel.connect_can_button.isEnabled(), "After CAN disconnect: Connect CAN button should be enabled."

    print("\n--- Parameter Panel Test End ---")
    sys.exit(app.exec())
