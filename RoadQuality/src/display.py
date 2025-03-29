import logging
import threading
import time
import smbus2
from src.sensors.lidar_sensor import LidarSensor
from src.sensors.gps_sensor import GPSSensor
from src.sensors.accelerometer import Accelerometer
from src.visualization.realtime_plots import RealTimePlots
from src.visualization.map_display import MapDisplay
from src.utils.config import Config

# Configure logging
logger = logging.getLogger("Display")
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

class Display:
    def __init__(self):
        self.config = Config()
        self.lidar = LidarSensor(port=self.config.LIDAR_PORT, scan_mode=self.config.LIDAR_SCAN_MODE)
        self.gps = GPSSensor(port=self.config.GPS_PORT, baud_rate=self.config.GPS_BAUD_RATE)
        
        # Initialize I2C bus
        try:
            self.i2c_bus = smbus2.SMBus(1)  # Use bus 1 for Raspberry Pi
            self.accelerometer = Accelerometer(i2c_bus=self.i2c_bus, address=self.config.ICM20948_ADDRESS)
        except Exception as e:
            logger.error(f"Failed to initialize I2C bus: {e}")
            self.i2c_bus = None
            self.accelerometer = None
        
        self.realtime_plots = RealTimePlots(max_data_points=self.config.MAX_DATA_POINTS)
        self.map_display = MapDisplay(initial_location=[0, 0], zoom_start=self.config.MAP_ZOOM_START)
        self.stop_event = threading.Event()
        self.threads = []

    def initialize_sensors(self):
        """Initialize all sensors."""
        try:
            logger.info("Initializing sensors...")
            lidar_initialized = self.lidar.initialize()
            gps_initialized = self.gps.initialize()
            accel_initialized = self.accelerometer.initialize() if self.accelerometer else False

            if not (lidar_initialized and gps_initialized and accel_initialized):
                logger.error("Failed to initialize one or more sensors.")
                return False

            logger.info("All sensors initialized successfully.")
            return True
        except Exception as e:
            logger.error(f"Error initializing sensors: {e}")
            return False

    def start_threads(self):
        """Start threads for data acquisition and visualization."""
        try:
            logger.info("Starting threads...")
            lidar_thread = threading.Thread(target=self.lidar_thread, daemon=True)
            gps_thread = threading.Thread(target=self.gps_thread, daemon=True)
            accel_thread = threading.Thread(target=self.accel_thread, daemon=True)

            self.threads.extend([lidar_thread, gps_thread, accel_thread])

            for thread in self.threads:
                thread.start()

            logger.info("All threads started successfully.")
        except Exception as e:
            logger.error(f"Error starting threads: {e}")

    def lidar_thread(self):
        """Thread function for LiDAR data acquisition."""
        while not self.stop_event.is_set():
            try:
                scan_data = self.lidar.get_scan_data()
                if scan_data:
                    angles, distances = zip(*scan_data)
                    self.realtime_plots.update_lidar_plot(angles, distances)
            except Exception as e:
                logger.error(f"Error in LiDAR thread: {e}")
            time.sleep(0.1)

    def gps_thread(self):
        """Thread function for GPS data acquisition."""
        while not self.stop_event.is_set():
            try:
                gps_data = self.gps.read_data()
                if gps_data:
                    self.map_display.add_marker(
                        location=[gps_data["lat"], gps_data["lon"]],
                        popup_text=f"Lat: {gps_data['lat']}, Lon: {gps_data['lon']}"
                    )
            except Exception as e:
                logger.error(f"Error in GPS thread: {e}")
            time.sleep(1)

    def accel_thread(self):
        """Thread function for accelerometer data acquisition."""
        while not self.stop_event.is_set():
            try:
                accel_data = self.accelerometer.get_acceleration()
                if accel_data:
                    self.realtime_plots.update_accel_plot([accel_data[2]])  # Z-axis acceleration
            except Exception as e:
                logger.error(f"Error in accelerometer thread: {e}")
            time.sleep(0.1)

    def stop_all(self):
        """Stop all sensors and threads."""
        try:
            logger.info("Stopping all sensors and threads...")
            self.stop_event.set()
            for thread in self.threads:
                thread.join()
            self.lidar.stop()
            self.gps.close()
            if hasattr(self, 'i2c_bus') and self.i2c_bus:
                self.i2c_bus.close()  # Close the I2C bus
            logger.info("All sensors and threads stopped successfully.")
        except Exception as e:
            logger.error(f"Error stopping sensors and threads: {e}")

    def run(self):
        """Main function to run the display."""
        try:
            if not self.initialize_sensors():
                logger.error("Failed to initialize sensors. Exiting...")
                return

            self.start_threads()
            self.realtime_plots.show()

        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt received. Stopping...")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            self.stop_all()

if __name__ == "__main__":
    display = Display()
    display.run()