import time

class LidarSensor:
    def __init__(self, port='/dev/ttyUSB0', scan_mode=0, max_distance=4000):
        self.port = port
        self.scan_mode = scan_mode
        self.max_distance = max_distance
        self.device = None

    def initialize(self):
        """Initialize the LiDAR sensor."""
        try:
            from fastestrplidar import FastestRplidar
            self.device = FastestRplidar()
            self.device.connectlidar()
            self.device.startmotor(my_scanmode=self.scan_mode)
            return True
        except ImportError:
            print("Failed to import FastestRplidar. Ensure the library is installed.")
            return False
        except Exception as e:
            print(f"Failed to initialize LiDAR: {e}")
            return False

    def get_scan_data(self):
        """Retrieve scan data from the LiDAR sensor."""
        if self.device is None:
            raise RuntimeError("LiDAR device not initialized.")
        
        try:
            scan_data = self.device.get_scan_as_vectors(filter_quality=True)
            return scan_data
        except Exception as e:
            print(f"Error retrieving scan data: {e}")
            return []

    def stop(self):
        """Stop the LiDAR sensor."""
        try:
            if self.device is not None:
                self.device.stopmotor()
                self.device.disconnectlidar()
        except Exception as e:
            print(f"Error stopping LiDAR: {e}")

    def reset(self):
        """Reset the LiDAR sensor."""
        try:
            self.stop()
            time.sleep(1)  # Wait for a second before reinitializing
            return self.initialize()
        except Exception as e:
            print(f"Failed to reset LiDAR: {e}")
            return False

    def is_connected(self):
        """Check if the LiDAR sensor is connected."""
        return self.device is not None

    def __del__(self):
        """Ensure the LiDAR device is stopped and disconnected on deletion."""
        self.stop()