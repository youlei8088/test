import struct
import numpy as np

# Constants based on firmware/hardware and mit手算.docx
P_MIN, P_MAX = -95.5, 95.5  # rad (Updated as per mit手算.docx)
V_MIN, V_MAX = -45.0, 45.0  # rad/s (Updated as per mit手算.docx)
T_MIN, T_MAX = -18.0, 18.0  # Nm (torque) - Assuming this remains the same or is covered by mit手算.docx if different
KP_MIN, KP_MAX = 0.0, 500.0 # N-m/rad - Assuming this remains the same or is covered by mit手算.docx if different
KD_MIN, KD_MAX = 0.0, 5.0   # N-m/(rad/s)

# These helper functions are crucial for converting floating-point physical values
# (like position, velocity, torque, gains) into fixed-point unsigned integers
# that can be packed into the limited bits of a CAN message, and vice-versa.
# The conversion relies on knowing the min/max range of the physical value and the number of bits allocated for it.

def float_to_uint(x, x_min, x_max, bits):
    """
    Converts a float to an unsigned integer, clamping to the specified range.

    Args:
        x (float): The float value to convert.
        x_min (float): The minimum value of the float's range.
        x_max (float): The maximum value of the float's range.
        bits (int): The number of bits available for the unsigned integer representation.

    Returns:
        int: The unsigned integer representation of the float.
    """
    span = x_max - x_min
    offset = x_min
    # Clamp the input float to the defined min/max range before conversion
    x = np.clip(x, x_min, x_max)
    # Perform the conversion: scale to [0, 1], then to [0, 2^bits - 1]
    return int(((x - offset) * (float((1 << bits) - 1))) / span)

def uint_to_float(x_int, x_min, x_max, bits):
    """
    Converts an unsigned integer back to a float, based on its original range and bit allocation.

    Args:
        x_int (int): The unsigned integer to convert.
        x_min (float): The minimum value of the original float's range.
        x_max (float): The maximum value of the original float's range.
        bits (int): The number of bits used for the unsigned integer representation.

    Returns:
        float: The converted float value.
    """
    span = x_max - x_min
    offset = x_min
    # Perform the reverse conversion: scale from [0, 2^bits - 1] to [0, 1], then to [x_min, x_max]
    return (float(x_int) * span / (float((1 << bits) - 1))) + offset

class MessagePacker:
    """
    Handles packing and unpacking of CAN messages for motor control,
    adhering to the specific byte format required by the GIM6010 firmware.
    This class uses the float_to_uint and uint_to_float helper functions.
    """
    def __init__(self, logger=None):
        """
        Initializes the MessagePacker.

        Args:
            logger (GUILogger, optional): Logger for outputting messages. Defaults to None (prints to console).
        """
        self.logger = logger

    def _log(self, message):
        """Internal helper to log messages via the provided logger or print."""
        if self.logger:
            self.logger.log(f"[MessagePacker] {message}")
        else:
            print(f"[MessagePacker] {message}")

    def pack_cmd(self, p_des, v_des, kp, kd, t_ff):
        """
        Packs motor command parameters into an 8-byte array for CAN transmission.
        This function must strictly follow the "new firmware" packing logic, which is
        assumed to be the standard MIT Cheetah / MGR Hobbysky format:
        - p_des (desired position, rad): 16 bits
        - v_des (desired velocity, rad/s): 12 bits
        - kp (position gain, Nm/rad): 12 bits
        - kd (velocity gain, Nm/(rad/s)): 12 bits
        - t_ff (feed-forward torque, Nm): 12 bits

        The byte order and bit arrangement are critical for correct firmware interpretation.

        Args:
            p_des (float): Desired position in radians.
            v_des (float): Desired velocity in rad/s.
            kp (float): Position (proportional) gain.
            kd (float): Velocity (derivative) gain.
            t_ff (float): Feed-forward torque in Nm.

        Returns:
            bytes: An 8-byte bytearray ready for CAN transmission.
                   Returns None if there's an error during value conversion (e.g., invalid input).
        """
        try:
            # Convert floating point values to scaled unsigned integers
            p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
            v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
            kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
            kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
            t_ff_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)
        except Exception as e:
            self._log(f"Error during float_to_uint conversion in pack_cmd: {e}")
            self._log(f"Input values: p_des={p_des}, v_des={v_des}, kp={kp}, kd={kd}, t_ff={t_ff}")
            return None

        # Pack the integers into an 8-byte buffer (bytearray)
        # This specific packing order is common for MIT Cheetah based controllers:
        # buf[0]: p_int_high_byte
        # buf[1]: p_int_low_byte
        # buf[2]: v_int_high_byte (top 8 of 12 bits)
        # buf[3]: v_int_low_4_bits | kp_int_high_4_bits
        # buf[4]: kp_int_low_byte (bottom 8 of 12 bits)
        # buf[5]: kd_int_high_byte (top 8 of 12 bits)
        # buf[6]: kd_int_low_4_bits | t_ff_int_high_4_bits
        # buf[7]: t_ff_int_low_byte (bottom 8 of 12 bits)

        buf = bytearray(8)
        buf[0] = (p_int >> 8) & 0xFF      # Position high byte
        buf[1] = p_int & 0xFF             # Position low byte
        buf[2] = (v_int >> 4) & 0xFF      # Velocity high byte (bits 11-4 of v_int)
        buf[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F) # Vel low 4 bits | Kp high 4 bits
        buf[4] = kp_int & 0xFF            # Kp low byte (bits 7-0 of kp_int)
        buf[5] = (kd_int >> 4) & 0xFF     # Kd high byte (bits 11-4 of kd_int)
        buf[6] = ((kd_int & 0x0F) << 4) | ((t_ff_int >> 8) & 0x0F) # Kd low 4 bits | T_ff high 4 bits
        buf[7] = t_ff_int & 0xFF          # T_ff low byte (bits 7-0 of t_ff_int)

        # Optional: Log the packed values for debugging
        # self._log(f"Packed CMD: p_des={p_des:.2f}({p_int}), v_des={v_des:.2f}({v_int}), kp={kp:.2f}({kp_int}), kd={kd:.2f}({kd_int}), t_ff={t_ff:.2f}({t_ff_int})")
        # self._log(f"Raw bytes: {[f'{b:02X}' for b in buf]}")
        return bytes(buf) # Convert bytearray to immutable bytes


    def unpack_reply(self, data):
        """
        Unpacks motor status data from a 6-byte CAN message payload.
        This must strictly follow the firmware's `pack_reply` logic.
        The assumed format (common for MIT Cheetah based controllers):
        - Byte 0: Motor ID
        - Bytes 1, 2: Actual position (p_int, 16 bits)
        - Bytes 3, 4 (first 4 bits of Byte 4): Actual velocity (v_int, 12 bits)
        - Bytes 4 (last 4 bits of Byte 4), 5: Actual torque (t_int, 12 bits)

        Args:
            data (bytes): A byte array (typically 6 bytes) received from CAN.

        Returns:
            tuple: (motor_id, position_rad, velocity_rad_s, torque_nm)
                   Returns None if unpacking fails (e.g., incorrect data length).
        """
        if not data or len(data) < 6: # Ensure enough data for standard reply
            self._log(f"Unpack failed: Incorrect data length {len(data) if data else 0}. Expected at least 6 bytes.")
            return None

        # Unpack data according to the assumed firmware reply format
        motor_id = data[0]
        p_int = (data[1] << 8) | data[2]                # Position (16-bit)
        v_int = (data[3] << 4) | (data[4] >> 4)         # Velocity (12-bit, from data[3] and high nibble of data[4])
        t_int = ((data[4] & 0x0F) << 8) | data[5]       # Torque (12-bit, from low nibble of data[4] and data[5])

        try:
            # Convert scaled unsigned integers back to floating point values
            pos_rad = uint_to_float(p_int, P_MIN, P_MAX, 16)
            vel_rad_s = uint_to_float(v_int, V_MIN, V_MAX, 12)
            torque_nm = uint_to_float(t_int, T_MIN, T_MAX, 12)
        except Exception as e:
            self._log(f"Error during uint_to_float conversion in unpack_reply: {e}")
            self._log(f"Raw ints: id={motor_id}, p_int={p_int}, v_int={v_int}, t_int={t_int}")
            return None

        # Optional: Log the unpacked values for debugging
        # self._log(f"Unpacked Reply: id={motor_id}, p={pos_rad:.2f} rad, v={vel_rad_s:.2f} rad/s, t={torque_nm:.2f} Nm")
        return motor_id, pos_rad, vel_rad_s, torque_nm

if __name__ == '__main__':
    # This section provides example usage and basic tests for the MessagePacker class.
    # It can be run directly to verify packing and unpacking logic.
    class SimpleTestLogger: # Minimal logger for testing
        def log(self, message):
            print(f"TEST_LOG: {message}")

    logger = SimpleTestLogger()
    packer = MessagePacker(logger=logger)

    # --- Test pack_cmd ---
    print("\n--- Testing pack_cmd ---")
    # Example command parameters (adjust as needed for realistic scenarios)
    p_des_test_rad = -1.0  # radians (approx -57 degrees)
    v_des_test_rad_s = 0.5   # rad/s
    kp_test_gain = 50.0    # Nm/rad
    kd_test_gain = 1.0     # Nm/(rad/s)
    t_ff_test_nm = 0.2     # Nm

    packed_cmd_bytes = packer.pack_cmd(p_des_test_rad, v_des_test_rad_s, kp_test_gain, kd_test_gain, t_ff_test_nm)
    if packed_cmd_bytes:
        # Print the hex representation of the packed bytes
        hex_bytes_str = " ".join([f"{b:02X}" for b in packed_cmd_bytes])
        print(f"Packed command (hex): {hex_bytes_str} (Length: {len(packed_cmd_bytes)})")

        # For verification, manually calculate expected integer values (approximate):
        # p_int_exp = float_to_uint(p_des_test_rad, P_MIN, P_MAX, 16) # e.g., ~30067 for P_MIN=-12.5
        # v_int_exp = float_to_uint(v_des_test_rad_s, V_MIN, V_MAX, 12) # e.g., ~2088 for V_MIN=-65.0
        # print(f"Expected intermediate ints (approx): p={p_int_exp}, v={v_int_exp}, ...")
    else:
        print("pack_cmd returned None (error in packing).")

    # Test with values at the limits (clamping should occur if outside P_MIN/MAX etc.)
    p_des_high_rad = 20.0 # This is outside the typical P_MAX of 12.5 rad
    packed_cmd_high_p = packer.pack_cmd(p_des_high_rad, v_des_test_rad_s, kp_test_gain, kd_test_gain, t_ff_test_nm)
    if packed_cmd_high_p:
        hex_bytes_high_p_str = " ".join([f"{b:02X}" for b in packed_cmd_high_p])
        print(f"Packed command (high p_des, clamped): {hex_bytes_high_p_str}")


    # --- Test unpack_reply ---
    print("\n--- Testing unpack_reply ---")
    # Create a sample packed reply byte string (simulating what the motor might send)
    # Example: motor_id=1, pos=0.1 rad, vel=-0.2 rad/s, torque=0.05 Nm
    # First, convert these to their integer representations:
    target_p_rad_reply = 0.1
    target_v_rad_s_reply = -0.2
    target_t_nm_reply = 0.05

    sample_p_int_reply = float_to_uint(target_p_rad_reply, P_MIN, P_MAX, 16)
    sample_v_int_reply = float_to_uint(target_v_rad_s_reply, V_MIN, V_MAX, 12)
    sample_t_int_reply = float_to_uint(target_t_nm_reply, T_MIN, T_MAX, 12)

    # Manually construct the 6-byte reply data based on the known unpacking format:
    sample_reply_data_bytes = bytes([
        1,  # Motor ID
        (sample_p_int_reply >> 8) & 0xFF,               # Position high byte
        sample_p_int_reply & 0xFF,                      # Position low byte
        (sample_v_int_reply >> 4) & 0xFF,               # Velocity high byte (bits 11-4)
        ((sample_v_int_reply & 0x0F) << 4) | \
        ((sample_t_int_reply >> 8) & 0x0F),             # Vel low 4 bits | Torque high 4 bits
        sample_t_int_reply & 0xFF                       # Torque low byte (bits 7-0)
    ])
    hex_reply_bytes_str = " ".join([f"{b:02X}" for b in sample_reply_data_bytes])
    print(f"Sample reply bytes (hex): {hex_reply_bytes_str}")

    unpacked_reply_data = packer.unpack_reply(sample_reply_data_bytes)
    if unpacked_reply_data:
        res_id, res_p, res_v, res_t = unpacked_reply_data
        print(f"Unpacked data: ID={res_id}, Pos={res_p:.4f} rad, Vel={res_v:.4f} rad/s, Torque={res_t:.4f} Nm")
        # Check if unpacked values are close to the original target values (accounting for quantization)
        assert abs(res_p - target_p_rad_reply) < 0.01, "Position unpack failed"
        assert abs(res_v - target_v_rad_s_reply) < 0.01, "Velocity unpack failed" # Vel has fewer bits, might have larger error
        assert abs(res_t - target_t_nm_reply) < 0.01, "Torque unpack failed"
        print("Unpack test successful (values are close to original).")
    else:
        print("unpack_reply returned None (error in unpacking).")

    # Test unpack_reply with insufficient data length
    print("\nTesting unpack_reply with short data:")
    packer.unpack_reply(bytes([1, 2, 3])) # Should log an error and return None

    # --- Test pack_cmd with values typical for GUI (0 degrees and -120 degrees) ---
    print("\n--- Testing pack_cmd with GUI-typical values ---")
    # Scenario 1: Target 0 degrees (0 radians)
    # Kp=10, Kd=0.1, T_ff=0
    p_gui_0_rad = 0.0
    packed_0_deg = packer.pack_cmd(p_gui_0_rad, 0.0, 10.0, 0.1, 0.0)
    if packed_0_deg:
        hex_0_deg_str = " ".join([f"{b:02X}" for b in packed_0_deg])
        print(f"Packed for 0 rad target: {hex_0_deg_str}")
        # Expected based on manual calculation in previous comments: 7F FF 7F F0 51 05 17 FF
        # (This depends on P_MIN/MAX, V_MIN/MAX etc. being symmetric around zero)

    # Scenario 2: Target -120 degrees (approx -2.094 radians)
    p_gui_neg120_rad = -120.0 * np.pi / 180.0
    packed_neg120_deg = packer.pack_cmd(p_gui_neg120_rad, 0.0, 10.0, 0.1, 0.0)
    if packed_neg120_deg:
        hex_neg120_deg_str = " ".join([f"{b:02X}" for b in packed_neg120_deg])
        print(f"Packed for -120deg ({p_gui_neg120_rad:.3f} rad) target: {hex_neg120_deg_str}")
        # Expected based on manual calculation: 6A 8F 7F F0 51 05 17 FF

    print("\n--- MessagePacker Test Finished ---")
    # Further checks:
    # - Ensure P_MIN, P_MAX etc. constants match firmware for accurate conversion.
    # - The problem states "新版固件 限制120度内". This typically refers to firmware's internal
    #   THETA_MIN/THETA_MAX limits, not necessarily a different CAN packing format.
    #   The packing format itself is usually general for the controller type.
    #   The GUI is responsible for sending p_des within the 0 to -120 degree range.
