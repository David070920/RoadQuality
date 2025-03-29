import logging

# Configure logging
logger = logging.getLogger("Accelerometer")

class Accelerometer:
    def __init__(self, i2c_bus, address=0x69):
        self.i2c_bus = i2c_bus
        self.address = address
        self.initialize()

    def initialize(self):
        """Initialize the accelerometer sensor."""
        try:
            who_am_i = self.read_byte(0x00)
            if who_am_i == 0xEA:
                self.i2c_bus.write_byte_data(self.address, 0x06, 0x00)  # Wake up the sensor
                return True
            else:
                raise Exception("Accelerometer not found.")
        except Exception as e:
            logger.error(f"Failed to initialize accelerometer: {e}")
            return False

    def is_initialized(self):
        """Check if the accelerometer is initialized."""
        try:
            return self.read_byte(0x00) == 0xEA
        except Exception:
            return False

    def read_byte(self, reg):
        """Read a byte from the accelerometer."""
        return self.i2c_bus.read_byte_data(self.address, reg)

    def read_word(self, reg):
        """Read a word from the accelerometer."""
        high = self.read_byte(reg)
        low = self.read_byte(reg + 1)
        return (high << 8) + low

    def read_word_2c(self, reg):
        """Read a 2's complement word from the accelerometer."""
        val = self.read_word(reg)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

    def get_acceleration(self):
        """Get acceleration data from the accelerometer."""
        try:
            accel_x = self.read_word_2c(0x3B) / 16384.0  # Convert to g
            accel_y = self.read_word_2c(0x3D) / 16384.0  # Convert to g
            accel_z = self.read_word_2c(0x3F) / 16384.0  # Convert to g
            return accel_x, accel_y, accel_z
        except Exception as e:
            logger.error(f"Failed to read acceleration data: {e}")
            return None, None, None

    def get_z_acceleration(self):
        """Get the Z-axis acceleration data."""
        return self.get_acceleration()[2]  # Return only Z-axis data

    def reset(self):
        """Reset the accelerometer."""
        try:
            return self.initialize()
        except Exception as e:
            logger.error(f"Failed to reset accelerometer: {e}")
            return False