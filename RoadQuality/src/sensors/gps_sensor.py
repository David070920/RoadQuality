import serial
import pynmea2
import logging
import time

# Configure logging
logger = logging.getLogger("GPSSensor")

class GPSSensor:
    def __init__(self, port='/dev/ttyACM0', baud_rate=9600, timeout=0.5):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_port = None
        self.gps_data = {"timestamp": None, "lat": 0, "lon": 0, "alt": 0, "sats": 0}

    def initialize(self):
        """Initialize the GPS sensor."""
        try:
            self.serial_port = serial.Serial(self.port, baudrate=self.baud_rate, timeout=self.timeout)
            logger.info(f"GPS initialized successfully on port {self.port} with baud rate {self.baud_rate}")
            return True
        except serial.SerialException as e:
            logger.error(f"SerialException: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to initialize GPS: {e}")
            return False

    def is_connected(self):
        """Check if the GPS sensor is connected."""
        return self.serial_port is not None and self.serial_port.is_open

    def read_data(self):
        """Read data from the GPS sensor."""
        try:
            raw_data = self.serial_port.readline().decode().strip()
            if 'GGA' in raw_data:
                gps_message = pynmea2.parse(raw_data)
                self.gps_data = {
                    "timestamp": gps_message.timestamp,
                    "lat": round(gps_message.latitude, 6),
                    "lon": round(gps_message.longitude, 6),
                    "alt": gps_message.altitude,
                    "sats": gps_message.num_sats
                }
                logger.info(f"GPS Data: {self.gps_data}")
                return self.gps_data
        except pynmea2.ParseError:
            logger.warning("Failed to parse GPS data.")
        except Exception as e:
            logger.error(f"Error reading GPS data: {e}")
        return None

    def close(self):
        """Close the GPS serial port."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info("GPS serial port closed")

    def reset(self):
        """Reset the GPS sensor."""
        try:
            self.close()
            time.sleep(1)  # Wait for a second before reinitializing
            return self.initialize()
        except Exception as e:
            logger.error(f"Failed to reset GPS: {e}")
            return False