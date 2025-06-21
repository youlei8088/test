from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider,
                             QSpinBox, QPushButton, QGroupBox, QDoubleSpinBox, QLCDNumber)
from PyQt6.QtCore import Qt, pyqtSignal
from test.utils import constants as const # Using absolute import from 'test'
import numpy as np

class MotionPanel(QWidget):
    """
    MotionPanel widget for controlling motor movement via CAN bus.
    This panel includes:
    - Real-time position control using a horizontal slider and an LCD display for the target angle.
    - Automated cycle testing controls: number of cycles, acceleration, max velocity, and a start/stop button.

    Signals are emitted for target angle changes and cycle test commands.
    """

    # --- Signals ---
    # Emitted when the target angle slider is changed by the user.
    # Carries the new target angle in degrees.
    targetAngleChanged = pyqtSignal(float)
    # Emitted when the "Start Cycle Test" button is clicked.
    # Carries the number of cycles, acceleration (rad/s^2), and max velocity (rad/s).
    startCycleTest = pyqtSignal(int, float, float)
    # Emitted when the "Stop Cycle Test" button (which was formerly "Start Cycle Test") is clicked.
    stopCycleTest = pyqtSignal()


    def __init__(self, parent=None):
        """
        Initializes the MotionPanel widget.
        Args:
            parent (QWidget, optional): The parent widget. Defaults to None.
        """
        super().__init__(parent)
        self.setObjectName("MotionPanel") # For styling or identification
        self._init_ui() # Setup the user interface elements
        self._is_cycling = False # Internal state to track if cycle test is active

    def _init_ui(self):
        """Sets up the layout and widgets for the motion panel."""
        main_layout = QVBoxLayout(self) # Main vertical layout

        # --- Real-time Position Control Group ---
        position_group = QGroupBox("Real-time Position Control (CAN)")
        position_layout = QVBoxLayout()

        # Slider and LCD for target angle
        angle_slider_layout = QHBoxLayout() # Horizontal layout for label, slider, LCD
        angle_label = QLabel("Target Angle (°):")

        self.angle_slider = QSlider(Qt.Orientation.Horizontal)
        # Slider configuration:
        # The slider's integer range (0 to SLIDER_ANGLE_STEPS) is mapped to the
        # float angle range (ANGLE_MAX_DEG to ANGLE_MIN_DEG, e.g., 0° to -120°).
        # Precision is determined by SLIDER_ANGLE_STEPS (e.g., 1200 steps for 0.1° precision over 120° range).
        self.angle_slider.setRange(0, const.SLIDER_ANGLE_STEPS)
        # Set initial slider position to correspond to ANGLE_MAX_DEG (typically 0°).
        initial_slider_value = self._degrees_to_slider_value(const.ANGLE_MAX_DEG)
        self.angle_slider.setValue(initial_slider_value)
        self.angle_slider.valueChanged.connect(self._on_angle_slider_changed) # Connect to slot
        self.angle_slider.setToolTip(f"Control target angle from {const.ANGLE_MAX_DEG}° to {const.ANGLE_MIN_DEG}°.")

        self.angle_lcd = QLCDNumber() # LCD to display the current target angle
        self.angle_lcd.setSegmentStyle(QLCDNumber.SegmentStyle.Flat) # Modern flat look
        self.angle_lcd.setDigitCount(5) # To display values like "-120.0"
        self.angle_lcd.display(self._slider_value_to_degrees(initial_slider_value)) # Show initial angle

        angle_slider_layout.addWidget(angle_label)
        angle_slider_layout.addWidget(self.angle_slider, 1) # Slider takes up more horizontal space
        angle_slider_layout.addWidget(self.angle_lcd)
        position_layout.addLayout(angle_slider_layout)

        position_group.setLayout(position_layout)
        main_layout.addWidget(position_group)

        # --- Automated Cycle Test Group ---
        cycle_test_group = QGroupBox("Automated Cycle Test (CAN)")
        cycle_test_layout = QGridLayout() # Grid layout for test parameters

        # Number of cycles input
        cycle_count_label = QLabel("Number of Cycles:")
        self.cycle_count_spinbox = QSpinBox()
        self.cycle_count_spinbox.setRange(1, 10000) # Min 1 cycle, max 10000
        self.cycle_count_spinbox.setValue(const.DEFAULT_CYCLE_COUNT)
        self.cycle_count_spinbox.setToolTip("Set the number of 0° to -120° (and back) cycles.")

        # Acceleration input
        accel_label = QLabel("Acceleration (rad/s²):")
        self.accel_spinbox = QDoubleSpinBox()
        self.accel_spinbox.setRange(0.1, 100.0) # Range for acceleration
        self.accel_spinbox.setSingleStep(0.1)
        self.accel_spinbox.setValue(const.DEFAULT_ACCELERATION)
        self.accel_spinbox.setDecimals(2) # Display two decimal places
        self.accel_spinbox.setToolTip("Set the acceleration for the automated cycle trajectory.")

        # Max velocity input
        max_vel_label = QLabel("Max Velocity (rad/s):")
        self.max_vel_spinbox = QDoubleSpinBox()
        self.max_vel_spinbox.setRange(0.1, 2 * np.pi * 5) # Example max: up to ~5 full cycles/sec velocity
        self.max_vel_spinbox.setSingleStep(0.1)
        self.max_vel_spinbox.setValue(const.DEFAULT_CYCLE_MAX_VELOCITY)
        self.max_vel_spinbox.setDecimals(2)
        self.max_vel_spinbox.setToolTip("Set the maximum velocity for the automated cycle trajectory.")

        # Start/Stop Cycle Test button
        self.start_cycle_button = QPushButton("Start Cycle Test")
        self.start_cycle_button.setToolTip("Begin automated cycling between 0° and -120° (or user-defined limits if changed).")
        self.start_cycle_button.clicked.connect(self._on_start_stop_cycle_clicked) # Connect to slot

        # Add cycle test widgets to the grid layout
        cycle_test_layout.addWidget(cycle_count_label, 0, 0)
        cycle_test_layout.addWidget(self.cycle_count_spinbox, 0, 1)
        cycle_test_layout.addWidget(accel_label, 1, 0)
        cycle_test_layout.addWidget(self.accel_spinbox, 1, 1)
        cycle_test_layout.addWidget(max_vel_label, 2, 0)
        cycle_test_layout.addWidget(self.max_vel_spinbox, 2, 1)
        cycle_test_layout.addWidget(self.start_cycle_button, 3, 0, 1, 2) # Button spans two columns

        cycle_test_group.setLayout(cycle_test_layout)
        main_layout.addWidget(cycle_test_group)

        # Motion controls are initially disabled; enabled upon successful CAN connection.
        self.set_motion_controls_enabled(False)

        main_layout.addStretch() # Push content to the top
        self.setLayout(main_layout)

    # --- Helper methods for slider value <-> degrees conversion ---
    def _slider_value_to_degrees(self, slider_val):
        """Converts an integer slider value to a float angle in degrees."""
        # Slider value maps linearly from 0 -> ANGLE_MAX_DEG to SLIDER_ANGLE_STEPS -> ANGLE_MIN_DEG
        percentage = float(slider_val) / const.SLIDER_ANGLE_STEPS
        angle_deg = const.ANGLE_MAX_DEG + percentage * (const.ANGLE_MIN_DEG - const.ANGLE_MAX_DEG)
        return round(angle_deg, 1) # Round to one decimal place for display consistency

    def _degrees_to_slider_value(self, degrees):
        """Converts a float angle in degrees to an integer slider value."""
        if (const.ANGLE_MIN_DEG - const.ANGLE_MAX_DEG) == 0: return 0 # Avoid division by zero if range is zero
        percentage = (degrees - const.ANGLE_MAX_DEG) / (const.ANGLE_MIN_DEG - const.ANGLE_MAX_DEG)
        slider_val = int(round(percentage * const.SLIDER_ANGLE_STEPS))
        # Clamp value to slider's valid range (0 to SLIDER_ANGLE_STEPS)
        return max(0, min(const.SLIDER_ANGLE_STEPS, slider_val))

    # --- Slots for UI interactions ---
    def _on_angle_slider_changed(self, slider_value):
        """Handles QSlider valueChanged signal: updates LCD and emits targetAngleChanged signal."""
        angle_deg = self._slider_value_to_degrees(slider_value)
        self.angle_lcd.display(angle_deg) # Update the LCD display
        self.targetAngleChanged.emit(angle_deg) # Emit signal with the new target angle

    def _on_start_stop_cycle_clicked(self):
        """Handles Start/Stop Cycle Test button click: emits appropriate signal."""
        if not self._is_cycling: # If not currently cycling, start the test
            cycles = self.cycle_count_spinbox.value()
            acceleration = self.accel_spinbox.value() # rad/s^2
            max_velocity = self.max_vel_spinbox.value() # rad/s
            self.startCycleTest.emit(cycles, acceleration, max_velocity)
            # Note: The main window is responsible for calling set_cycle_test_running(True)
            # after successfully starting the cycle test thread.
        else: # If currently cycling, stop the test
            self.stopCycleTest.emit()
            # Note: The main window calls set_cycle_test_running(False) upon test completion or stop.

    # --- Public methods for external interaction ---
    def get_target_angle_degrees(self):
        """Returns the current target angle in degrees as set by the slider."""
        return self._slider_value_to_degrees(self.angle_slider.value())

    def set_target_angle_degrees(self, degrees):
        """
        Programmatically sets the target angle slider and LCD.
        Useful for restoring state or after a cycle test.
        Blocks slider signals during programmatic change to avoid emitting targetAngleChanged.
        Args:
            degrees (float): The angle in degrees to set.
        """
        slider_val = self._degrees_to_slider_value(degrees)

        self.angle_slider.blockSignals(True) # Prevent emitting valueChanged during programmatic set
        self.angle_slider.setValue(slider_val)
        self.angle_slider.blockSignals(False) # Re-enable signals

        self.angle_lcd.display(self._slider_value_to_degrees(slider_val)) # Update LCD

    def set_motion_controls_enabled(self, enabled):
        """
        Enables or disables all motion control widgets.
        Called by the main window based on CAN connection status.
        Args:
            enabled (bool): True to enable, False to disable.
        """
        self.angle_slider.setEnabled(enabled)
        # self.angle_lcd.setEnabled(enabled) # LCDs are typically always enabled for display
        self.cycle_count_spinbox.setEnabled(enabled)
        self.accel_spinbox.setEnabled(enabled)
        self.max_vel_spinbox.setEnabled(enabled)
        self.start_cycle_button.setEnabled(enabled)

        # If controls are being disabled while a cycle test was running, reset button text.
        if not enabled and self._is_cycling:
            self.set_cycle_test_running(False) # Update button text and internal state

    def set_cycle_test_running(self, is_running):
        """
        Updates the UI to reflect the state of the automated cycle test.
        Called by the main window when the cycle test starts or stops.
        Args:
            is_running (bool): True if cycle test is active, False otherwise.
        """
        self._is_cycling = is_running
        if self._is_cycling:
            self.start_cycle_button.setText("Stop Cycle Test")
            self.start_cycle_button.setStyleSheet("background-color: orange;") # Visual cue
            # Disable manual angle slider and cycle parameters during automated test
            self.angle_slider.setEnabled(False)
            self.cycle_count_spinbox.setEnabled(False)
            self.accel_spinbox.setEnabled(False)
            self.max_vel_spinbox.setEnabled(False)
        else:
            self.start_cycle_button.setText("Start Cycle Test")
            self.start_cycle_button.setStyleSheet("") # Reset style
            # Re-enable controls if the parent group (and thus CAN connection) is active.
            # This check ensures controls aren't wrongly enabled if CAN got disconnected
            # while the cycle test was finishing.
            if self.isEnabled(): # Checks if this widget itself is enabled (depends on CAN status)
                 self.angle_slider.setEnabled(True)
                 self.cycle_count_spinbox.setEnabled(True)
                 self.accel_spinbox.setEnabled(True)
                 self.max_vel_spinbox.setEnabled(True)

if __name__ == '__main__':
    # This block allows testing the MotionPanel widget independently.
    import sys
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    # Create and show the MotionPanel instance for testing
    test_motion_panel = MotionPanel()

    # --- Test Slot Functions ---
    def simulate_handle_target_angle(angle_deg):
        print(f"TEST SLOT: TargetAngleChanged to {angle_deg:.1f}°")

    def simulate_handle_start_cycle(cycles, accel, max_v):
        print(f"TEST SLOT: StartCycleTest - Cycles={cycles}, Accel={accel:.2f} rad/s², MaxV={max_v:.2f} rad/s")
        # Simulate main window confirming cycle start
        test_motion_panel.set_cycle_test_running(True)

    def simulate_handle_stop_cycle():
        print("TEST SLOT: StopCycleTest requested")
        # Simulate main window confirming cycle stop
        test_motion_panel.set_cycle_test_running(False)

    # Connect signals to test slots
    test_motion_panel.targetAngleChanged.connect(simulate_handle_target_angle)
    test_motion_panel.startCycleTest.connect(simulate_handle_start_cycle)
    test_motion_panel.stopCycleTest.connect(simulate_handle_stop_cycle)

    test_motion_panel.setWindowTitle("Motion Panel - Standalone Test")
    test_motion_panel.show()

    # --- Automated Test Steps ---
    print("\n--- Motion Panel Test Steps ---")
    # 1. Initial state: controls should be disabled.
    print(f"Initial: Angle slider enabled? {test_motion_panel.angle_slider.isEnabled()} (Expected: False)")
    assert not test_motion_panel.angle_slider.isEnabled(), "Initial: Slider should be disabled."
    print(f"Initial: Start Cycle button enabled? {test_motion_panel.start_cycle_button.isEnabled()} (Expected: False)")
    assert not test_motion_panel.start_cycle_button.isEnabled(), "Initial: Start Cycle button should be disabled."

    # 2. Simulate enabling controls (e.g., after CAN connection in main app)
    print("\nStep 2: Simulating enabling motion controls...")
    test_motion_panel.set_motion_controls_enabled(True)
    print(f"After enable: Angle slider enabled? {test_motion_panel.angle_slider.isEnabled()} (Expected: True)")
    assert test_motion_panel.angle_slider.isEnabled(), "After enable: Slider should be enabled."

    # 3. Test slider to degrees mapping and signal emission
    print("\nStep 3: Testing slider mapping and targetAngleChanged signal...")
    # Test extremes and midpoint
    slider_val_0_deg = test_motion_panel._degrees_to_slider_value(0.0)
    assert slider_val_0_deg == 0, "Slider value for 0.0 deg incorrect."
    assert test_motion_panel._slider_value_to_degrees(0) == 0.0, "Degrees for slider value 0 incorrect."

    slider_val_neg120_deg = test_motion_panel._degrees_to_slider_value(-120.0)
    assert slider_val_neg120_deg == const.SLIDER_ANGLE_STEPS, "Slider value for -120.0 deg incorrect."
    assert test_motion_panel._slider_value_to_degrees(const.SLIDER_ANGLE_STEPS) == -120.0, "Degrees for max slider value incorrect."

    slider_val_neg60_deg = test_motion_panel._degrees_to_slider_value(-60.0)
    assert slider_val_neg60_deg == const.SLIDER_ANGLE_STEPS / 2, "Slider value for -60.0 deg incorrect."
    assert test_motion_panel._slider_value_to_degrees(const.SLIDER_ANGLE_STEPS / 2) == -60.0, "Degrees for mid slider value incorrect."

    # Simulate user moving slider to -90 degrees (approx)
    print("  Simulating user moving slider to value for -90 degrees...")
    target_slider_val_neg90 = test_motion_panel._degrees_to_slider_value(-90.0)
    test_motion_panel.angle_slider.setValue(target_slider_val_neg90)
    # Check LCD and that signal was emitted (verified by print in simulate_handle_target_angle)
    print(f"  LCD display after slider move: {test_motion_panel.angle_lcd.value():.1f}° (Expected: -90.0°)")
    assert abs(test_motion_panel.angle_lcd.value() - (-90.0)) < 0.01, "LCD not updated correctly after slider move."

    # 4. Test programmatic setting of angle
    print("\nStep 4: Programmatically setting target angle to -30.5 degrees...")
    test_motion_panel.set_target_angle_degrees(-30.5)
    print(f"  LCD display after programmatic set: {test_motion_panel.angle_lcd.value():.1f}° (Expected: -30.5°)")
    assert abs(test_motion_panel.angle_lcd.value() - (-30.5)) < 0.01, "LCD not updated by programmatic set."
    # Note: set_target_angle_degrees blocks signals, so simulate_handle_target_angle should not have printed.

    # 5. Test cycle test button functionality
    print("\nStep 5: Testing cycle test button...")
    test_motion_panel.start_cycle_button.click() # Should emit startCycleTest signal
    print(f"  After clicking Start: Button text='{test_motion_panel.start_cycle_button.text()}' (Expected: Stop Cycle Test)")
    assert test_motion_panel._is_cycling, "Internal state _is_cycling not True after start."
    assert "Stop" in test_motion_panel.start_cycle_button.text(), "Button text not updated to Stop."
    print(f"  Angle slider enabled during cycle test? {test_motion_panel.angle_slider.isEnabled()} (Expected: False)")
    assert not test_motion_panel.angle_slider.isEnabled(), "Slider should be disabled during cycle test."

    test_motion_panel.start_cycle_button.click() # Should emit stopCycleTest signal
    print(f"  After clicking Stop: Button text='{test_motion_panel.start_cycle_button.text()}' (Expected: Start Cycle Test)")
    assert not test_motion_panel._is_cycling, "Internal state _is_cycling not False after stop."
    assert "Start" in test_motion_panel.start_cycle_button.text(), "Button text not reset to Start."
    print(f"  Angle slider enabled after cycle test? {test_motion_panel.angle_slider.isEnabled()} (Expected: True)")
    assert test_motion_panel.angle_slider.isEnabled(), "Slider should be re-enabled after cycle test."

    # 6. Simulate disabling controls (e.g., after CAN disconnect)
    print("\nStep 6: Simulating disabling motion controls...")
    test_motion_panel.set_motion_controls_enabled(False)
    print(f"After disable: Angle slider enabled? {test_motion_panel.angle_slider.isEnabled()} (Expected: False)")
    assert not test_motion_panel.angle_slider.isEnabled(), "After disable: Slider should be disabled."

    print("\n--- Motion Panel Test End ---")
    sys.exit(app.exec())
