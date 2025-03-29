import threading
import logging
import signal
import sys
import os
import time
import webbrowser
import matplotlib.pyplot as plt
from collections import deque  # Import deque

from config import Config
from initialization import initialize_i2c, initialize_lidar, initialize_gps, initialize_icm20948
from data_acquisition import lidar_thread_func, gps_thread_func, accel_thread_func
from visualization import setup_visualization
from utils import update_gps_map, create_default_map

# Fix Wayland error
os.environ["QT_QPA_PLATFORM"] = "xcb"  # Use X11 instead of Wayland

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("SensorFusion")

class SensorFusion:
    def __init__(self):
        self.config = Config()
        
        # Log user and session information
        logger.info(f"Starting SensorFusion - User: {self.config.USER_LOGIN}, Session start: {self.config.SYSTEM_START_TIME}")
        
        # Data structures with thread safety
        self.lidar_data_lock = threading.Lock()
        self.lidar_data = []
        
        self.accel_data_lock = threading.Lock()
        # Changed: Use deque instead of list
        self.accel_data = deque(maxlen=self.config.MAX_DATA_POINTS)
        
        self.gps_data_lock = threading.Lock()
        # Changed: Update structure to match display.py
        self.gps_data = {"timestamp": None, "lat": 0, "lon": 0, "alt": 0, "sats": 0, "lock": self.gps_data_lock}
        self.last_map_update = 0  # Added: Track last map update time
        
        # Device handles
        self.lidar_device = None
        self.gps_serial_port = None
        self.i2c_bus = None
        
        # Thread control
        self.stop_event = threading.Event()
        self.threads = []
        
        # Visualization objects
        self.fig_lidar = None
        self.fig_accel = None
        self.lidar_ani = None
        self.accel_ani = None
        
        # Log the map file location
        logger.info(f"GPS map will be saved to: {self.config.MAP_HTML_PATH}")

    def initialize_devices(self):
        """Initialize all the devices"""
        self.i2c_bus = initialize_i2c()
        if not self.i2c_bus:
            logger.error("Failed to initialize I2C. Exiting.")
            return False
        
        self.lidar_device = initialize_lidar(self.config)
        if not self.lidar_device:
            logger.error("Failed to initialize LiDAR. Exiting.")
            return False
            
        self.gps_serial_port = initialize_gps(self.config)
        if not self.gps_serial_port:
            logger.warning("Failed to initialize GPS. Continuing without GPS.")
            
        if not initialize_icm20948(self.i2c_bus, self.config):
            logger.warning("Failed to initialize ICM20948. Continuing without accelerometer data.")
        
        return True

    def start_threads(self):
        """Start data acquisition threads"""
        # Changed: Pass the last_map_update as an object attribute
        self.threads = [
            threading.Thread(target=lidar_thread_func, args=(self.lidar_device, self.lidar_data_lock, self.lidar_data, self.stop_event, self.config), daemon=True),
            threading.Thread(target=gps_thread_func, 
                            args=(self.gps_serial_port, self.gps_data_lock, self.gps_data, 
                                self.stop_event, self.config, update_gps_map, self), daemon=True),
            threading.Thread(target=accel_thread_func, args=(self.i2c_bus, self.accel_data_lock, self.accel_data, self.stop_event, self.config), daemon=True)
        ]
        
        for thread in self.threads:
            thread.start()

    def setup_signal_handler(self):
        """Set up signal handler for graceful shutdown"""
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        """Handle SIGINT (Ctrl+C) gracefully"""
        logger.info("Shutdown signal received. Cleaning up...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Clean up resources before exit"""
        # Signal threads to stop
        self.stop_event.set()
        
        # Wait for threads to complete
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
                
        # Clean up device resources
        if self.lidar_device:
            try:
                self.lidar_device.stopmotor()
                logger.info("LiDAR motor stopped")
            except Exception as e:
                logger.error(f"Error stopping LiDAR motor: {e}")
                
        if self.gps_serial_port:
            try:
                self.gps_serial_port.close()
                logger.info("GPS serial port closed")
            except Exception as e:
                logger.error(f"Error closing GPS serial port: {e}")
                
        if self.i2c_bus:
            try:
                self.i2c_bus.close()
                logger.info("I2C bus closed")
            except Exception as e:
                logger.error(f"Error closing I2C bus: {e}")
                
        logger.info("Cleanup complete")

    def run(self):
        """Main function to run the application"""
        self.setup_signal_handler()
        
        if not self.initialize_devices():
            self.cleanup()
            return
        
        # Create default map before starting threads
        create_default_map(self.config)  # Added: Create default map
        
        self.start_threads()
        
        try:
            logger.info("Setting up visualization...")
            self.fig_lidar, self.fig_accel, self.lidar_ani, self.accel_ani = setup_visualization(
                self.lidar_data, self.lidar_data_lock, 
                self.accel_data, self.accel_data_lock, 
                self.config
            )
            
            # Try to open the map in browser - Added this section
            try:
                map_url = 'file://' + os.path.abspath(self.config.MAP_HTML_PATH)
                logger.info(f"Opening map at: {map_url}")
                if webbrowser.open(map_url):
                    logger.info("Map opened in browser")
                else:
                    logger.warning("Failed to open browser, but map file was created")
            except Exception as e:
                logger.error(f"Error opening map in browser: {e}")
            
            # Use plt.ioff() to avoid keeping windows always on top
            plt.ioff()
            # Show the figures but don't block
            plt.show(block=False)
            
            # Keep the main thread alive but responsive to signals
            while not self.stop_event.is_set():
                plt.pause(0.1)  # Update plots while allowing other operations
                
        except Exception as e:
            logger.error(f"Error in visualization: {e}")
        finally:
            self.cleanup()

if __name__ == '__main__':
    sensor_fusion = SensorFusion()
    sensor_fusion.run()
