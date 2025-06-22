from PyQt6.QtCore import QObject, pyqtSignal

class GUILogger(QObject):
    messageLogged = pyqtSignal(str)

    def __init__(self, text_edit_widget=None, max_lines=200):
        """
        Initializes the GUILogger.

        Args:
            text_edit_widget (QTextEdit, optional): The QTextEdit widget to log messages to. Defaults to None.
            max_lines (int, optional): Maximum number of lines to keep in the log. Defaults to 200.
        """
        super().__init__()
        self.text_edit = text_edit_widget
        self.max_lines = max_lines
        self._log_buffer = [] # Buffer for messages logged before a QTextEdit is connected

    def set_text_edit(self, text_edit_widget):
        """
        Sets the QTextEdit widget for logging and flushes any buffered messages.
        This is typically called by connect_to_widget.

        Args:
            text_edit_widget (QTextEdit): The QTextEdit widget.
        """
        self.text_edit = text_edit_widget
        # If there are buffered messages, display them now in the newly connected widget
        if self.text_edit and self._log_buffer:
            for msg in self._log_buffer:
                self._append_message_to_widget(msg) # Append them respecting max_lines
            self._log_buffer = []


    def _append_message_to_widget(self, message):
        """
        Appends a message to the connected QTextEdit widget, respecting max_lines.
        This method should be called from the GUI thread (e.g., via a signal).

        Args:
            message (str): The message string to append.
        """
        if not self.text_edit:
            return # No widget to log to

        # QTextEdit's appendPlainText is efficient, but we need to manage line count manually
        # to avoid performance degradation with very large documents.
        # A simpler way to manage lines without reading all text:
        doc = self.text_edit.document()
        current_block_count = doc.blockCount()

        if current_block_count >= self.max_lines:
            # Remove oldest lines to make space for the new one
            # Move cursor to the start, select lines to remove, then remove selection
            cursor = self.text_edit.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            for _ in range(current_block_count - self.max_lines + 1):
                cursor.movePosition(cursor.MoveOperation.EndOfBlock, cursor.MoveMode.KeepAnchor, 1) # Select one block (line)
                if cursor.atEnd() and not cursor.hasSelection(): # Avoid issues if somehow at end without selection
                    break
            cursor.removeSelectedText()
            cursor.movePosition(cursor.MoveOperation.EndOfBlock) # Go to end of remaining text
            self.text_edit.setTextCursor(cursor)


        self.text_edit.append(message) # Changed from appendPlainText to append
        # self.text_edit.ensureCursorVisible() # Alternative way to scroll to bottom

    def log(self, message):
        """
        Logs a message.
        Emits a signal to allow thread-safe updates to the GUI.
        If no QTextEdit is connected yet, the message is buffered.

        Args:
            message (str): The message to log.
        """
        self.messageLogged.emit(message) # Signal connected to _append_message_to_widget

        if not self.text_edit: # If no widget is connected yet, buffer the message
            if len(self._log_buffer) >= self.max_lines:
                self._log_buffer.pop(0) # Remove oldest if buffer itself is full
            self._log_buffer.append(message)

    def connect_to_widget(self, text_edit_widget):
        """
        Connects the logger to a QTextEdit widget.
        Messages logged via self.log() will be appended to this widget.
        This method ensures thread-safe updates by connecting the messageLogged signal.

        Args:
            text_edit_widget (QTextEdit): The QTextEdit widget to log messages to.
        """
        self.set_text_edit(text_edit_widget) # Set the widget and flush buffer
        # Connect the signal to the slot that updates the GUI widget
        # This ensures that _append_message_to_widget is called in the GUI thread
        if not hasattr(self, '_signal_connected') or not self._signal_connected: # Connect only once
            self.messageLogged.connect(self._append_message_to_widget)
            self._signal_connected = True


if __name__ == '__main__':
    # GUILogger is a QObject, ideally tested within a QApplication for full signal/slot behavior.
    # This basic test checks buffering and direct appending logic.
    # This is a QObject, so it's best tested within a Qt application loop
    # For a simple non-GUI test of the line limiting logic:
    class MockTextCursor:
        def __init__(self, doc):
            self.doc = doc
            self.pos = 0
            self.anchor = 0
        def movePosition(self, op, mode=None, n=None): # Simplified
            if op == MockTextCursor.MoveOperation.Start: self.pos = 0
            elif op == MockTextCursor.MoveOperation.EndOfBlock: self.pos += 1 # Very simplified
            if mode == MockTextCursor.MoveMode.KeepAnchor: pass # Anchor logic not fully mocked
            else: self.anchor = self.pos
        def removeSelectedText(self): pass # Simplified
        def atEnd(self): return self.pos >= self.doc.blockCount()
        def hasSelection(self): return self.pos != self.anchor
        MoveOperation = type('Enum', (), {'Start':1, 'EndOfBlock':2})
        MoveMode = type('Enum', (), {'KeepAnchor':1})


    class MockQTextDocument:
        def __init__(self): self.blocks = []
        def blockCount(self): return len(self.blocks)

    class MockQTextEdit:
        def __init__(self):
            self.document_obj = MockQTextDocument()
            self.plain_text = []
        def document(self): return self.document_obj
        def textCursor(self): return MockTextCursor(self.document_obj) # Simplified
        def setPlainText(self, text): # Not used by _append_message_to_widget directly for removal
            self.plain_text = text.splitlines()
            self.document_obj.blocks = self.plain_text[:]
        def appendPlainText(self, message):
            self.plain_text.append(message)
            self.document_obj.blocks.append(message)
            # print(f"MockAppended: {message}")
        def toPlainText(self): return "\n".join(self.plain_text)


    print("Testing GUILogger logic (simplified, no Qt event loop):")
    logger = GUILogger(max_lines=3)
    mock_widget = MockQTextEdit()

    # Simulate connecting the widget (signal connection part is Qt specific)
    logger.set_text_edit(mock_widget) # Manually set for this test, connect_to_widget also calls this.

    print("Logging initial messages (buffered before connect_to_widget actually connects signal):")
    logger.log("Msg1 (buffered)") # Goes to buffer
    logger.log("Msg2 (buffered)") # Goes to buffer

    # Simulate signal emission after connection (which calls _append_message_to_widget)
    # Normally, connect_to_widget would ensure messageLogged signal calls _append_message_to_widget
    # For this test, we'll manually call what the signal would do.
    # First, flush the buffer by calling set_text_edit again (as if connect_to_widget was just called)
    logger.set_text_edit(mock_widget)
    print(f"After buffer flush: {mock_widget.toPlainText()}")

    print("\nLogging more messages to test max_lines:")
    logger._append_message_to_widget("Msg3") # Simulating signal emission
    print(f"Content: {mock_widget.toPlainText()}")
    logger._append_message_to_widget("Msg4") # Simulating signal emission, Msg1 should be gone
    print(f"Content: {mock_widget.toPlainText()}")
    logger._append_message_to_widget("Msg5") # Simulating signal emission, Msg2 should be gone
    print(f"Content: {mock_widget.toPlainText()}")

    # Expected: Msg3, Msg4, Msg5 (if max_lines = 3 and removal logic works perfectly)
    # The mock text removal is very basic, so this test mainly shows intent.
    # Real QTextEdit line removal is more complex.
    # The current GUILogger._append_message_to_widget uses QTextDocument.blockCount()
    # and cursor operations, which are hard to mock simply.
    # The important part is that it attempts to limit lines.

    # Test buffering logic again
    logger_buffered = GUILogger(max_lines=2)
    logger_buffered.log("Buffer1")
    logger_buffered.log("Buffer2")
    logger_buffered.log("Buffer3") # Buffer1 should be popped
    assert len(logger_buffered._log_buffer) == 2
    assert logger_buffered._log_buffer[0] == "Buffer2"
    assert logger_buffered._log_buffer[1] == "Buffer3"
    print("\nBuffer test successful.")
    print("GUILogger basic test finished.")
