# Exoskeleton Motor Control GUI

This application provides a Python-based graphical user interface (GUI) for controlling, monitoring, and debugging a knee exoskeleton system based on an MD300 driver board and a GIM6010 direct-drive motor.

## Project Overview

The GUI allows for:
- **System Control (via Serial Port):** Enabling/disabling the motor, setting the zero position, and emergency stop.
- **Motion Control (via CAN Bus):** Real-time position control using a slider and automated cyclic testing between defined angle limits (0° to -120°).
- **Parameter Adjustment (via CAN Bus):** Real-time tuning of Proportional (Kp) and Derivative (Kd) gains, and Feed-forward Torque (T_ff).
- **Data Display:** Numerical feedback of actual motor angle, speed, and torque.
- **Graphical Visualization:**
    - A 2-link (thigh/shank) diagram animasi based on actual motor angle.
    - Real-time plots of target angle vs. actual motor angle.
- **Debug Logging:** A text area displaying CAN bus status, commands, received data, and error messages.

## Hardware & Firmware Requirements

- **Motor Controller:** MD300 driver board (DRV8323RS).
- **Motor:** GIM6010-6_24_GDM6 direct-drive motor (14 pole pairs, 1:1 gear ratio).
- **Position Sensor:** MA702 magnetic encoder.
- **PC-to-CAN Interface:** CANalyst-II USB-CAN analyzer (ZLG compatible). Requires `ControlCAN.dll`.
- **Firmware:** Assumes new version of manufacturer firmware for direct-drive motor with software position limits (THETA_MIN, THETA_MAX) and specific CAN/Serial communication protocols.

## Software Requirements

- **Python:** Version 3.6 or higher.
- **Required Python Libraries:**
    - `PyQt6`: For the graphical user interface.
    - `pyqtgraph`: For real-time plotting.
    - `pyserial`: For serial port communication.
    - `numpy`: For numerical operations (e.g., trajectory planning, data handling).
- **CAN Driver:** `ControlCAN.dll` (specific to the CANalyst-II device). This DLL must be placed in a location where the application can find it. The application will search in:
    1. The current working directory when the script is run.
    2. The `test/` directory (relative to the script's CWD, if running from project root).
    3. The directory containing `can_handler.py` (i.e., `test/can_communication/`).
    4. The directory containing the main application executable if bundled (e.g., with PyInstaller).
    It's generally recommended to place `ControlCAN.dll` inside the `test/` directory or alongside `main.py` if running directly. If building an executable, include it in the bundle.

## Running the Application

1.  **Ensure all software requirements are met.** Install Python libraries using pip:
    ```bash
    pip install PyQt6 pyqtgraph pyserial numpy
    ```
2.  **Place `ControlCAN.dll`** in an accessible location (e.g., within the `test` directory, or the same directory as `main.py`).
3.  **Navigate to the project's root directory** (the directory containing the `test` folder).
4.  **Run the application as a module:**
    ```bash
    python -m test.main
    ```
    Alternatively, navigate into the `test` directory and run:
    ```bash
    python main.py
    ```
    (The `main.py` script includes some path adjustments to try and handle this, but running as a module from the project root is generally more robust for package-like structures.)

## Key Features & Usage Notes

-   **Serial Connection:**
    -   Select the correct COM port for your CP210x device from the dropdown.
    -   Click "Refresh" to update the list of available ports.
    -   Click "Connect Serial" to establish communication. Motor commands are enabled after successful connection.
-   **CAN Connection:**
    -   Click "Connect CAN" in the Parameter Panel to initialize the CANalyst-II device. Motion controls and parameter tuning are enabled after successful connection.
    -   The application assumes a CAN baud rate of 1Mbps.
-   **Emergency Stop:**
    -   This button (in the Control Panel) sends a disable command via serial and attempts to set CAN motor commands to safe values (zero gains, hold current position). It's a critical safety feature.
-   **Real-time Position Control:**
    -   Use the slider in the Motion Panel to set the target motor angle (0° to -120°).
    -   The motor should follow the slider position in real-time if CAN is connected and the motor is enabled.
-   **Automated Cycle Test:**
    -   Set the number of cycles, acceleration, and maximum velocity in the Motion Panel.
    -   Click "Start Cycle Test" to make the motor automatically move between 0° and -120°.
    -   Click "Stop Cycle Test" to interrupt the test.
-   **Parameter Tuning:**
    -   Use the dials or spinboxes in the Parameter Panel to adjust Kp, Kd, and T_ff.
    -   Changes take effect immediately if CAN is connected.
-   **Angle Conventions:**
    -   The GUI displays angles from 0° (fully extended/reference) to -120° (maximum flexion).
    -   Internally, these are converted to radians for calculations and CAN commands.
-   **CAN Command Frequency:**
    -   Position/PD commands are sent periodically (default: every 20ms / 50Hz) via CAN to maintain control and prevent firmware timeouts. This is handled by the `CANCommunicationThread`.
-   **Logging:**
    -   The "Debug Log" area shows status messages, errors, and summaries of CAN/Serial activity.

## Code Structure

-   `main.py`: Entry point of the application.
-   `gui/`: Contains all PyQt6 UI components.
    -   `main_window.py`: The main application window, orchestrating all parts.
    -   `widgets/`: Individual panels for control, motion, parameters, and display.
        -   `control_panel.py`: Serial controls.
        -   `motion_panel.py`: CAN-based motion control and cycle test.
        -   `param_panel.py`: CAN connection and PD parameter tuning.
        -   `display_panel.py`: Numerical, graphical (joint diagram, plots), and log displays.
-   `can_communication/`: Logic for CAN bus communication.
    -   `can_handler.py`: Interface to `ControlCAN.dll` using `ctypes`.
    -   `message_packer.py`: Packs and unpacks CAN messages according to firmware specification.
-   `serial_communication/`: Logic for serial port communication.
    -   `serial_handler.py`: Interface using `pyserial`.
-   `utils/`: Utility modules.
    -   `constants.py`: Application-wide constants.
    -   `logger.py`: GUI-compatible logger.
-   `assets/`: (Currently a placeholder) For images or other static assets.
-   `ControlCAN.dll`: (User-provided) The necessary DLL for CANalyst-II.

## Known Limitations / Future Work

-   **Hardware Dependency:** Requires specific CAN hardware (CANalyst-II) and its DLL.
-   **Single Motor Focus:** Currently designed and tested for controlling a single motor.
-   **CAN ID Configuration:** The CAN ID for commanding the motor (`MOTOR_CMD_CAN_ID`) is currently hardcoded in `utils/constants.py`. This might need to be configurable if multiple motors or different firmware defaults are used. The interpretation of "CAN_MASTER_ID" for replies also relies on assumptions that need firmware verification.
-   **Error Handling:** While basic error messages are provided, comprehensive error recovery (e.g., automatic CAN reconnect attempts) is not implemented.
-   **Trajectory Planner:** The cycle test uses a simple trapezoidal/triangular velocity profile. More advanced profiles could be implemented.
-   **Parameter Saving:** Tuned PD parameters are not saved between sessions.
-   **Extensive Hardware Testing:** The application has been developed based on specifications; rigorous testing on the actual hardware setup is crucial.
```
