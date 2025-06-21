import sys
from PyQt6.QtWidgets import QApplication

# Ensure the current directory is in PYTHONPATH if running from project root (e.g. `python test/main.py`)
# This helps with relative imports within the 'test' package if 'test' itself is not installed.
# However, if 'test' is intended to be run as a module (python -m test.main), this might not be needed
# or could even be problematic depending on how Python resolves 'test'.
# For robustness when running `python test/main.py` from one level up:
import os
# current_dir = os.path.dirname(os.path.abspath(__file__)) # This is /path/to/project/test
# project_root = os.path.dirname(current_dir) # This is /path/to/project
# if project_root not in sys.path:
#    sys.path.insert(0, project_root)
# Now imports like `from test.gui.main_window import MainWindow` should work if `test` is the top package.

# If running with `python -m test.main`, then relative imports like `from .gui.main_window` are preferred.
# Let's assume the structure is:
# project_root/
#   test/
#     main.py
#     gui/
#     can_communication/
#     ...
# And we run `python -m test.main` from `project_root` or `python main.py` from `project_root/test/`.

try:
    from .gui.main_window import MainWindow
    from .utils.logger import GUILogger # Though MainWindow instantiates its own
    from .utils import constants as const
except ImportError as e:
    print(f"ImportError: {e}. This might be due to how the script is run.")
    print("If running from the project root, try 'python -m test.main'.")
    print("If running directly from the 'test' directory, ensure your PYTHONPATH is set up correctly or use 'python -m main'.")
    # Fallback for running script directly from `test` directory for dev convenience
    if __package__ is None or __package__ == '':
        print("Attempting to adjust sys.path for direct script execution...")
        package_dir = os.path.dirname(os.path.abspath(__file__)) # .../test
        project_root_dir = os.path.dirname(package_dir) # .../
        if project_root_dir not in sys.path:
            sys.path.insert(0, project_root_dir)
        # Now, the imports should work as if 'test' is a package.
        # e.g. from test.gui.main_window import MainWindow
        # This requires renaming the imports below if they were relative.
        # For now, the original relative imports are kept, assuming module execution.
        # Let's make them absolute from the perspective of 'test' being a top-level package.
        from gui.main_window import MainWindow
        from utils.logger import GUILogger
        from utils import constants as const
        print("sys.path adjusted. Imports re-attempted.")
    else:
        raise e # Re-raise if it's already being treated as a package and still fails


def main():
    """
    Main function to initialize and run the PyQt6 application.
    """
    app = QApplication(sys.argv)

    # Optional: Apply a global style
    # app.setStyle("Fusion")

    # The MainWindow class already initializes its own logger, serial_handler, can_handler.
    # It also connects signals and sets up the UI.
    main_win = MainWindow()
    main_win.show()

    # The GUILogger instance is created within MainWindow.
    # If a global logger instance were needed for other modules outside MainWindow,
    # it could be instantiated here and passed around, but MainWindow's internal logger
    # is sufficient for GUI logging.
    # Example:
    # global_logger = GUILogger()
    # global_logger.connect_to_widget(main_win.display_panel.get_debug_text_edit())
    # global_logger.log("Application started via main.py")
    # (MainWindow already logs its start)

    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("Application terminated by user (KeyboardInterrupt).")
        # QApplication should handle cleanup, but MainWindow.closeEvent also does.
        # main_win.close() # Ensure settings are saved, threads are stopped. closeEvent handles this.
        sys.exit(1)


if __name__ == "__main__":
    # This structure allows `python test/main.py` or `python -m test.main`
    # The ImportError handling above tries to make direct execution `python main.py` from `test/` work.
    main()
