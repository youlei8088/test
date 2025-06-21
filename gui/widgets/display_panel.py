from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGroupBox,
                             QTextEdit, QFrame)
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QPolygonF, QFont
from PyQt6.QtCore import Qt, QPointF, pyqtSignal, QRectF
import pyqtgraph as pg
import numpy as np
from test.utils import constants as const # Using absolute import from 'test'

class JointDiagramWidget(QWidget):
    """
    JointDiagramWidget is a custom QWidget that draws a simple 2-link (thigh and shank)
    representation of the exoskeleton leg. The angle of the knee joint is updated
    based on motor feedback.
    """
    def __init__(self, parent=None):
        """
        Initializes the JointDiagramWidget.
        Args:
            parent (QWidget, optional): The parent widget. Defaults to None.
        """
        super().__init__(parent)
        self.setMinimumSize(200, 200) # Ensure a decent default size for the diagram
        self.angle_rad = 0.0 # Current knee angle in radians (0 = straight, negative = flexion)

        # Define segment lengths and origin (can be made configurable or dynamic)
        self.origin_x = const.JOINT_ORIGIN_X    # X-coordinate of the hip joint
        self.origin_y = const.JOINT_ORIGIN_Y    # Y-coordinate of the hip joint
        self.thigh_len = const.THIGH_LENGTH     # Length of the thigh segment
        self.shank_len = const.SHANK_LENGTH     # Length of the shank segment

        # Colors for drawing
        self.thigh_color = QColor(*const.THIGH_COLOR)
        self.shank_color = QColor(*const.SHANK_COLOR)
        self.hip_joint_color = QColor(*const.HIP_COLOR)
        self.knee_joint_color = QColor(*const.KNEE_COLOR)
        self.joint_radius = 5 # Radius for drawing joint circles


    def set_angle(self, angle_degrees):
        """
        Sets the knee angle for the diagram.
        The input angle is expected to be in degrees, where the GUI uses 0° for
        straight and negative values (e.g., -120°) for flexion.
        This is converted to radians for trigonometric calculations.

        Args:
            angle_degrees (float): The knee joint angle in degrees.
        """
        # The problem defines angle range as 0 to -120 degrees for the motor.
        # For drawing, a negative angle_rad will represent flexion in typical kinematic diagrams.
        self.angle_rad = angle_degrees * const.DEG_TO_RAD
        self.update() # Request a repaint of the widget

    def paintEvent(self, event):
        """
        Handles the paint event to draw the joint diagram.
        The diagram consists of a fixed hip joint, a thigh segment, a knee joint,
        and a shank segment whose angle is determined by self.angle_rad.

        Args:
            event (QPaintEvent): The paint event.
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing) # For smooth lines

        # Fill background (optional, can make it transparent or match theme)
        painter.fillRect(self.rect(), QColor(245, 245, 245)) # Light grey background

        # Hip Joint (Origin) - fixed point for the diagram
        hip_x, hip_y = self.origin_x, self.origin_y

        # Thigh Segment (Segment 1)
        # Assume thigh is angled slightly downwards from horizontal for a natural side view.
        # This angle is fixed for simplicity in this diagram.
        thigh_angle_from_horizontal_rad = 15 * const.DEG_TO_RAD # e.g., 15 degrees down

        thigh_end_x = hip_x + self.thigh_len * np.cos(thigh_angle_from_horizontal_rad)
        thigh_end_y = hip_y + self.thigh_len * np.sin(thigh_angle_from_horizontal_rad) # Positive y is downwards

        # Shank Segment (Segment 2)
        # The shank angle is relative to the thigh's orientation plus the knee flexion angle.
        # self.angle_rad is the knee flexion (0 for straight, negative for flexion).
        # So, total angle of shank from horizontal = thigh_angle + knee_flexion_angle.
        shank_angle_rad = thigh_angle_from_horizontal_rad + self.angle_rad

        shank_end_x = thigh_end_x + self.shank_len * np.cos(shank_angle_rad)
        shank_end_y = thigh_end_y + self.shank_len * np.sin(shank_angle_rad)

        # --- Draw the segments ---
        # Thigh
        pen = QPen(self.thigh_color, 10, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap) # Thick, round-ended line
        painter.setPen(pen)
        painter.drawLine(QPointF(hip_x, hip_y), QPointF(thigh_end_x, thigh_end_y))

        # Shank
        pen.setColor(self.shank_color) # Change color for shank
        painter.setPen(pen)
        painter.drawLine(QPointF(thigh_end_x, thigh_end_y), QPointF(shank_end_x, shank_end_y))

        # --- Draw the joints (circles) ---
        # Hip Joint
        painter.setBrush(QBrush(self.hip_joint_color))
        pen.setColor(Qt.GlobalColor.darkGray) # Outline for joints
        pen.setWidth(1) # Thinner outline
        painter.setPen(pen)
        painter.drawEllipse(QPointF(hip_x, hip_y), self.joint_radius, self.joint_radius)

        # Knee Joint
        painter.setBrush(QBrush(self.knee_joint_color))
        painter.drawEllipse(QPointF(thigh_end_x, thigh_end_y), self.joint_radius, self.joint_radius)


class DisplayPanel(QWidget):
    """
    DisplayPanel widget for showing various forms of feedback from the motor.
    This includes:
    - Numerical display of actual angle, speed, and torque.
    - A graphical JointDiagramWidget for visualizing leg posture.
    - A pyqtgraph PlotWidget for real-time plotting of target vs. actual angles.
    - A QTextEdit area for debug log messages.
    """
    def __init__(self, parent=None):
        """
        Initializes the DisplayPanel.
        Args:
            parent (QWidget, optional): The parent widget. Defaults to None.
        """
        super().__init__(parent)
        self.setObjectName("DisplayPanel") # For styling or identification
        self._init_ui() # Setup UI elements

        # Data storage for plots (circular buffers essentially, managed by truncation)
        self.time_data = np.array([])           # Time axis data
        self.target_angle_data = np.array([])   # Target angle data for plot
        self.actual_angle_data = np.array([])   # Actual angle data for plot
        self.max_plot_points = const.PLOT_UPDATE_MS * 20 * 5 # e.g., 5 seconds of data if PLOT_UPDATE_MS is 50ms (20Hz) -> 100 points
                                                            # Let's make it simpler, e.g., 200 points
        self.max_plot_points = 300 # Show last N data points on the plot

    def _init_ui(self):
        """Sets up the layout and widgets for the display panel."""
        main_layout = QVBoxLayout(self) # Main vertical layout

        # --- Numerical Display Group ---
        numerical_group = QGroupBox("Real-time Feedback")
        numerical_layout = QGridLayout() # Grid for organized label-value pairs

        # Actual Angle Display
        self.angle_label = QLabel("Actual Angle (°):")
        self.angle_value = QLabel("N/A") # Placeholder text
        self.angle_value.setFont(QFont("Consolas", 14, QFont.Weight.Bold)) # Monospaced, bold font
        self.angle_value.setFrameStyle(QFrame.Shape.Panel | QFrame.Shadow.Sunken) # Sunken panel look
        self.angle_value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.angle_value.setMinimumWidth(80) # Ensure enough space

        # Actual Speed Display
        self.speed_label = QLabel("Actual Speed (°/s):")
        self.speed_value = QLabel("N/A")
        self.speed_value.setFont(QFont("Consolas", 14, QFont.Weight.Bold))
        self.speed_value.setFrameStyle(QFrame.Shape.Panel | QFrame.Shadow.Sunken)
        self.speed_value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.speed_value.setMinimumWidth(80)

        # Actual Torque Display
        self.torque_label = QLabel("Actual Torque (Nm):")
        self.torque_value = QLabel("N/A")
        self.torque_value.setFont(QFont("Consolas", 14, QFont.Weight.Bold))
        self.torque_value.setFrameStyle(QFrame.Shape.Panel | QFrame.Shadow.Sunken)
        self.torque_value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.torque_value.setMinimumWidth(80)

        # Add numerical display widgets to the grid
        numerical_layout.addWidget(self.angle_label, 0, 0)
        numerical_layout.addWidget(self.angle_value, 0, 1)
        numerical_layout.addWidget(self.speed_label, 1, 0)
        numerical_layout.addWidget(self.speed_value, 1, 1)
        numerical_layout.addWidget(self.torque_label, 2, 0)
        numerical_layout.addWidget(self.torque_value, 2, 1)
        numerical_group.setLayout(numerical_layout)
        main_layout.addWidget(numerical_group)

        # --- Graphical Display Area (Joint Diagram on left, Plot on right) ---
        graphical_area_layout = QHBoxLayout() # Horizontal layout for diagram and plot

        # Joint Diagram section
        joint_diagram_group = QGroupBox("Joint Diagram")
        joint_diagram_layout = QVBoxLayout()
        self.joint_diagram_widget = JointDiagramWidget() # Instantiate custom widget
        joint_diagram_layout.addWidget(self.joint_diagram_widget)
        joint_diagram_group.setLayout(joint_diagram_layout)
        graphical_area_layout.addWidget(joint_diagram_group, 1) # Diagram takes 1 part of stretch factor

        # Plot Area section (using pyqtgraph)
        plot_group = QGroupBox("Angle Plot (Target vs. Actual)")
        plot_layout = QVBoxLayout()
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w') # White background for the plot
        self.plot_widget.setLabel('left', 'Angle', units='°') # Y-axis label
        self.plot_widget.setLabel('bottom', 'Time', units='s') # X-axis label
        self.plot_widget.addLegend() # Show legend for plot lines
        self.plot_widget.showGrid(x=True, y=True) # Display grid lines

        # Define plot lines for target and actual angles
        self.target_angle_curve = self.plot_widget.plot(name="Target Angle", pen=pg.mkPen(const.TARGET_ANGLE_COLOR, width=2))
        self.actual_angle_curve = self.plot_widget.plot(name="Actual Angle", pen=pg.mkPen(const.ACTUAL_ANGLE_COLOR, width=2))

        plot_layout.addWidget(self.plot_widget)
        plot_group.setLayout(plot_layout)
        graphical_area_layout.addWidget(plot_group, 3) # Plot takes 3 parts of stretch factor (larger)

        main_layout.addLayout(graphical_area_layout)


        # --- Debug Feedback Area ---
        debug_group = QGroupBox("Debug Log")
        debug_layout = QVBoxLayout()
        self.debug_text_edit = QTextEdit() # For displaying log messages
        self.debug_text_edit.setReadOnly(True) # User cannot edit the log
        self.debug_text_edit.setFixedHeight(120) # Set a fixed height for the debug area
        debug_layout.addWidget(self.debug_text_edit)
        debug_group.setLayout(debug_layout)
        main_layout.addWidget(debug_group)

        self.setLayout(main_layout) # Set the main layout for the DisplayPanel

    # --- Public methods for updating displays ---
    def update_numerical_display(self, angle_deg, speed_deg_s, torque_nm):
        """
        Updates the numerical QLabel widgets with new motor feedback data.
        Args:
            angle_deg (float): Actual motor angle in degrees.
            speed_deg_s (float): Actual motor speed in degrees per second.
            torque_nm (float): Actual motor torque in Newton-meters.
        """
        self.angle_value.setText(f"{angle_deg:.1f}")
        self.speed_value.setText(f"{speed_deg_s:.1f}")
        self.torque_value.setText(f"{torque_nm:.2f}")

    def update_joint_diagram(self, angle_deg):
        """
        Updates the joint diagram with the new actual motor angle.
        Args:
            angle_deg (float): Actual motor angle in degrees.
        """
        self.joint_diagram_widget.set_angle(angle_deg)

    def update_plot(self, time_s, target_angle_deg, actual_angle_deg):
        """
        Adds new data points to the real-time plot and updates the display.
        Manages a fixed-size window of data points by truncating old data.
        Args:
            time_s (float): Current time in seconds for the x-axis.
            target_angle_deg (float): Target motor angle in degrees.
            actual_angle_deg (float): Actual motor angle in degrees.
        """
        # Manage data buffer size: if max_plot_points is reached, remove the oldest point
        if len(self.time_data) >= self.max_plot_points:
            self.time_data = np.delete(self.time_data, 0)
            self.target_angle_data = np.delete(self.target_angle_data, 0)
            self.actual_angle_data = np.delete(self.actual_angle_data, 0)

        # Append new data points
        self.time_data = np.append(self.time_data, time_s)
        self.target_angle_data = np.append(self.target_angle_data, target_angle_deg)
        self.actual_angle_data = np.append(self.actual_angle_data, actual_angle_deg)

        # Set plot data for both curves
        # To make the plot scroll, we can adjust the X-axis range dynamically.
        # If time_data is always increasing, pyqtgraph might handle this automatically.
        # Alternatively, set a fixed view window:
        if len(self.time_data) > 1:
             self.plot_widget.setXRange(max(self.time_data[0], self.time_data[-1] - (self.max_plot_points * (const.PLOT_UPDATE_MS/1000.0))), self.time_data[-1])

        self.target_angle_curve.setData(self.time_data, self.target_angle_data)
        self.actual_angle_curve.setData(self.time_data, self.actual_angle_data)

    def get_debug_text_edit(self):
        """Returns the QTextEdit widget used for debug logging."""
        return self.debug_text_edit

    def clear_plots(self):
        """Clears all data from the plot curves and resets the data arrays."""
        self.time_data = np.array([])
        self.target_angle_data = np.array([])
        self.actual_angle_data = np.array([])

        # Clear data from plot items
        self.target_angle_curve.setData([], []) # Pass empty arrays
        self.actual_angle_curve.setData([], [])

        self.plot_widget.autoRange() # Adjust plot view to be empty or default

if __name__ == '__main__':
    # This block allows testing the DisplayPanel widget independently.
    import sys
    from PyQt6.QtWidgets import QApplication
    from PyQt6.QtCore import QTimer

    app = QApplication(sys.argv)
    app = QApplication(sys.argv)

    # Create and show the DisplayPanel instance for testing
    test_display_panel = DisplayPanel()
    test_display_panel.setWindowTitle("Display Panel - Standalone Test")
    test_display_panel.show()

    # --- Test Data Update Examples ---
    print("Updating numerical display and joint diagram with initial test data...")
    test_display_panel.update_numerical_display(angle_deg=-30.5, speed_deg_s=15.2, torque_nm=-1.55)
    test_display_panel.update_joint_diagram(angle_deg=-30.5) # Diagram shows -30.5 deg flexion

    # Simulate live plot updates using a QTimer
    test_start_time = pg.time() # Get a high-resolution start time for plot

    def simulate_timed_plot_update():
        """Function called by QTimer to simulate new data points for the plot."""
        current_sim_time = pg.time() - test_start_time # Time in seconds since test start

        # Generate some example data (e.g., sine wave for target, noisy sine for actual)
        target_angle = -60 + 60 * np.sin(current_sim_time * 1.5) # Oscillating target
        actual_angle = target_angle - 8 * np.random.rand() + 4   # Actual angle with some noise/lag

        # Update the plot
        test_display_panel.update_plot(current_sim_time, target_angle, actual_angle)

        # Also update numerical displays and joint diagram with the 'actual' data
        # Simulate some speed and torque based on actual angle for variety
        sim_speed = (target_angle - actual_angle) * 10 # Dummy speed calculation
        sim_torque = actual_angle / -20.0              # Dummy torque calculation
        test_display_panel.update_numerical_display(actual_angle, sim_speed, sim_torque)
        test_display_panel.update_joint_diagram(actual_angle)

    # Setup a QTimer to call the update function periodically
    plot_update_timer = QTimer()
    plot_update_timer.timeout.connect(simulate_timed_plot_update)
    plot_update_timer.start(const.PLOT_UPDATE_MS) # Update plot at defined interval (e.g., 50ms)
    print(f"Simulating live plot updates every {const.PLOT_UPDATE_MS} ms...")

    # Test the debug log area
    debug_log_widget = test_display_panel.get_debug_text_edit()
    debug_log_widget.append("Debug log test: Initial message.")
    debug_log_widget.append("Debug log test: Another message to check scrolling and line limits.")
    for i in range(5): # Add a few more lines
        debug_log_widget.append(f"Debug log test: Log line {i+1}.")

    # Test clearing plots after a few seconds (optional)
    def simulate_clear_plots():
        print("\nClearing plots now (simulated action)...")
        test_display_panel.clear_plots()
        # Note: If you want to restart plotting after clearing for testing,
        # you'd need to reset test_start_time or handle time data array re-initialization.
        # For this test, clearing is enough to see the effect.
        # global test_start_time # If you were to reset it
        # test_start_time = pg.time()

    # QTimer.singleShot(5000, simulate_clear_plots) # Example: clear plots after 5 seconds.
    # Commented out to let plot run continuously for manual observation.

    print("\nDisplayPanel test is running. Observe the GUI for updates.")
    print("Close the 'Display Panel - Standalone Test' window to exit.")
    sys.exit(app.exec())
