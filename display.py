import serial
import time
import pynmea2
import threading
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from fastestrplidar import FastestRplidar
import smbus2
from collections import deque
import logging
import signal
import sys
import os
import folium
import webbrowser
from pathlib import Path
from datetime import datetime

# Fix Wayland error
os.environ["QT_QPA_PLATFORM"] = "xcb"  # Use X11 instead of Wayland

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("SensorFusion")

class SensorFusion:
    # Configuration class
    class Config:
        # General settings
        MAX_DATA_POINTS = 100  # Maximum data points to store in memory
        UPDATE_INTERVAL = 10   # Visualization update interval in ms
        GPS_MAP_UPDATE_INTERVAL = 10   # GPS map update interval in seconds
        
        # User and system information
        USER_LOGIN = "David070920"
        SYSTEM_START_TIME = "2025-03-29 11:17:21"
        
        # LiDAR settings
        LIDAR_PORT = '/dev/ttyUSB0'
        LIDAR_DMAX = 4000      # Maximum LiDAR distance
        LIDAR_SCAN_MODE = 0    # Scan mode (0-2)
        
        # Modified: Changed to capture the specific 90-degree cone (315-360 and 0-45 degrees)
        LIDAR_MIN_ANGLE = -45  # Minimum display angle (converted from 315° to -45° for polar plot)
        LIDAR_MAX_ANGLE = 45   # Maximum display angle
        LIDAR_FILTER_ANGLES = [(0, 45), (315, 360)]  # Angles to keep (min, max)
        
        # GPS settings
        GPS_PORT = '/dev/ttyACM0'
        GPS_BAUD_RATE = 9600
        GPS_TIMEOUT = 0.5
        
        # ICM20948 settings
        ICM20948_ADDRESS = 0x69
        ICM20948_WHO_AM_I = 0x00
        ICM20948_PWR_MGMT_1 = 0x06
        ICM20948_ACCEL_ZOUT_H = 0x31
        
        # Folium map settings - Using absolute path in home directory
        MAP_HTML_PATH = os.path.join(str(Path.home()), "gps_position.html")
        MAP_ZOOM_START = 15
    
    def __init__(self):
        self.config = self.Config()
        
        # Log user and session information
        logger.info(f"Starting SensorFusion - User: {self.config.USER_LOGIN}, Session start: {self.config.SYSTEM_START_TIME}")
        
        # Data structures with thread safety
        self.lidar_data_lock = threading.Lock()
        self.lidar_data = []
        
        self.accel_data_lock = threading.Lock()
        self.accel_data = deque(maxlen=self.config.MAX_DATA_POINTS)
        
        self.gps_data_lock = threading.Lock()
        self.gps_data = {"timestamp": None, "lat": 0, "lon": 0, "alt": 0, "sats": 0}
        self.last_map_update = 0
        
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

    def initialize_i2c(self):
        """Initialize the I2C bus"""
        try:
            self.i2c_bus = smbus2.SMBus(1)
            time.sleep(0.1)
            return True
        except Exception as e:
            logger.error(f"Failed to initialize I2C: {e}")
            return False

    def initialize_lidar(self):
        """Initialize the LiDAR sensor"""
        try:
            self.lidar_device = FastestRplidar()
            self.lidar_device.connectlidar()
            self.lidar_device.startmotor(my_scanmode=self.config.LIDAR_SCAN_MODE)
            logger.info("LiDAR initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize LiDAR: {e}")
            return False

    def initialize_gps(self):
        """Initialize the GPS module"""
        try:
            self.gps_serial_port = serial.Serial(
                self.config.GPS_PORT, 
                baudrate=self.config.GPS_BAUD_RATE, 
                timeout=self.config.GPS_TIMEOUT
            )
            logger.info("GPS initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize GPS: {e}")
            return False

    def initialize_icm20948(self):
        """Initialize the ICM20948 accelerometer"""
        try:
            who_am_i = self.read_byte(self.config.ICM20948_ADDRESS, self.config.ICM20948_WHO_AM_I)
            
            if who_am_i == 0xEA:
                logger.info(f"ICM20948 found at address 0x{self.config.ICM20948_ADDRESS:02x}")
                # Wake up the sensor (clear sleep mode)
                self.i2c_bus.write_byte_data(
                    self.config.ICM20948_ADDRESS, 
                    self.config.ICM20948_PWR_MGMT_1, 
                    0x00
                )
                time.sleep(0.1)
                return True
            else:
                logger.error(f"ICM20948 WHO_AM_I register mismatch. Expected 0xEA, got {who_am_i}")
                return False
        except Exception as e:
            logger.error(f"Failed to initialize ICM20948: {e}")
            return False

    def read_byte(self, addr, reg):
        """Read a byte from the I2C device"""
        retries = 3
        for _ in range(retries):
            try:
                return self.i2c_bus.read_byte_data(addr, reg)
            except Exception as e:
                logger.debug(f"Error reading byte from address 0x{addr:02x}, register 0x{reg:02x}: {e}")
                time.sleep(0.01)
        logger.warning(f"Failed to read from address 0x{addr:02x}, register 0x{reg:02x} after {retries} retries")
        return None

    def read_word(self, addr, reg):
        """Read a word from the I2C device"""
        high = self.read_byte(addr, reg)
        low = self.read_byte(addr, reg + 1)
        if high is not None and low is not None:
            val = (high << 8) + low
            return val
        else:
            return None

    def read_word_2c(self, addr, reg):
        """Read a 2's complement word from the I2C device"""
        val = self.read_word(addr, reg)
        if val is not None:
            if val >= 0x8000:
                return -((65535 - val) + 1)
            else:
                return val
        else:
            return None

    def get_accel_data(self):
        """Get accelerometer data from ICM20948"""
        accel_z = self.read_word_2c(self.config.ICM20948_ADDRESS, self.config.ICM20948_ACCEL_ZOUT_H)
        if accel_z is not None:
            return accel_z / 16384.0  # Convert to g
        return None

    def filter_lidar_angles(self, scan_data):
        """Filter LiDAR data to only include points within specified angle ranges"""
        filtered_data = []
        
        for point in scan_data:
            angle = point[0]
            for angle_range in self.config.LIDAR_FILTER_ANGLES:
                if angle_range[0] <= angle <= angle_range[1]:
                    filtered_data.append(point)
                    break
                    
        return filtered_data

    def lidar_thread_func(self):
        """Thread function for LiDAR data acquisition"""
        logger.info("LiDAR thread started")
        while not self.stop_event.is_set():
            try:
                # Fetch the LiDAR scan data
                scan_data = self.lidar_device.get_scan_as_vectors(filter_quality=True)
                
                # Filter data based on angles
                filtered_data = self.filter_lidar_angles(scan_data)
                
                # Update shared data with lock
                with self.lidar_data_lock:
                    self.lidar_data = filtered_data
                    
            except Exception as e:
                logger.error(f"Error in LiDAR thread: {e}")
                
            # Sleep to prevent high CPU usage
            time.sleep(0.05)
        
        logger.info("LiDAR thread stopped")

    def gps_thread_func(self):
        """Thread function for GPS data acquisition"""
        logger.info("GPS thread started")
        while not self.stop_event.is_set():
            try:
                # Fetch the GPS data
                raw_data = self.gps_serial_port.readline().decode().strip()
                
                if raw_data.find('GGA') > 0:
                    gps_message = pynmea2.parse(raw_data)
                    
                    # Update shared data with lock
                    with self.gps_data_lock:
                        self.gps_data = {
                            "timestamp": gps_message.timestamp,
                            "lat": round(gps_message.latitude, 6),
                            "lon": round(gps_message.longitude, 6),
                            "alt": gps_message.altitude,
                            "sats": gps_message.num_sats
                        }
                    
                    # Check if it's time to update the map
                    current_time = time.time()
                    if current_time - self.last_map_update >= self.config.GPS_MAP_UPDATE_INTERVAL:
                        self.update_gps_map()
                        self.last_map_update = current_time
                    
                    logger.info(f"GPS: {self.gps_data}")
                    
            except Exception as e:
                logger.debug(f"Error in GPS thread: {e}")
                
            # Sleep to prevent high CPU usage
            time.sleep(0.2)
            
        logger.info("GPS thread stopped")

    def update_gps_map(self):
        """Update the GPS position on a Folium map and save as HTML"""
        try:
            with self.gps_data_lock:
                lat = self.gps_data["lat"]
                lon = self.gps_data["lon"]
                alt = self.gps_data["alt"]
                sats = self.gps_data["sats"]
                timestamp = self.gps_data["timestamp"]
            
            # Skip if we don't have valid coordinates yet
            if lat == 0 and lon == 0:
                logger.warning("No valid GPS coordinates yet, skipping map update")
                return
                
            # Create a map centered at the GPS coordinates
            m = folium.Map(location=[lat, lon], zoom_start=self.config.MAP_ZOOM_START)
            
            # Add a marker for the current position
            popup_text = f"""
            <b>GPS Data</b><br>
            Latitude: {lat:.6f}°<br>
            Longitude: {lon:.6f}°<br>
            Altitude: {alt} m<br>
            Satellites: {sats}<br>
            Time: {timestamp}<br>
            <hr>
            User: {self.config.USER_LOGIN}<br>
            Session: {self.config.SYSTEM_START_TIME}
            """
            
            folium.Marker(
                [lat, lon], 
                popup=folium.Popup(popup_text, max_width=300)
            ).add_to(m)
            
            # Add a circle to show accuracy (just for visualization)
            folium.Circle(
                location=[lat, lon],
                radius=10,  # 10 meters radius
                color='blue',
                fill=True,
                fill_opacity=0.2
            ).add_to(m)
            
            # Save the map to an HTML file
            m.save(self.config.MAP_HTML_PATH)
            logger.info(f"GPS map updated at {self.config.MAP_HTML_PATH}")
            
            # Verify the file exists
            if os.path.exists(self.config.MAP_HTML_PATH):
                logger.info(f"GPS map file successfully created")
            else:
                logger.error(f"Failed to create GPS map file")
                
        except Exception as e:
            logger.error(f"Error updating GPS map: {e}")

    def create_default_map(self):
        """Create a default map if no GPS data is available yet"""
        try:
            # Create a map centered at a default location (0, 0 or another predefined location)
            m = folium.Map(location=[0, 0], zoom_start=2)
            
            # Add explanatory text with user and session information
            popup_text = f"""
            <b>Waiting for GPS data...</b><br>
            <hr>
            User: {self.config.USER_LOGIN}<br>
            Session: {self.config.SYSTEM_START_TIME}
            """
            
            folium.Marker(
                [0, 0], 
                popup=folium.Popup(popup_text, max_width=300),
                icon=folium.Icon(color='red')
            ).add_to(m)
            
            # Save the map
            m.save(self.config.MAP_HTML_PATH)
            logger.info(f"Default map created at {self.config.MAP_HTML_PATH}")
            
        except Exception as e:
            logger.error(f"Error creating default map: {e}")

    def accel_thread_func(self):
        """Thread function for accelerometer data acquisition"""
        logger.info("Accelerometer thread started")
        while not self.stop_event.is_set():
            try:
                # Get accelerometer data
                accel_z = self.get_accel_data()
                
                if accel_z is not None:
                    # Update shared data with lock
                    with self.accel_data_lock:
                        self.accel_data.append(accel_z)
                    
                    logger.debug(f"Accelerometer: Z={accel_z:.2f}g")
                    
            except Exception as e:
                logger.error(f"Error in accelerometer thread: {e}")
                
            # Sleep to prevent high CPU usage
            time.sleep(0.1)
            
        logger.info("Accelerometer thread stopped")

    def update_lidar_plot(self, num, line):
        """Update function for LiDAR animation"""
        with self.lidar_data_lock:
            if not self.lidar_data:
                return line,
                
            # Process the data for visualization - 
            # Convert angles to the format expected by the polar plot
            polar_data = []
            for point in self.lidar_data:
                angle_deg = point[0]
                distance = point[1]
                
                # Convert 315-360 degrees to -45-0 degrees for the polar plot
                if angle_deg >= 315 and angle_deg <= 360:
                    angle_deg = angle_deg - 360
                
                # Only include angles in our desired range
                if -45 <= angle_deg <= 45:
                    polar_data.append((np.radians(angle_deg), distance))
            
            if not polar_data:
                return line,
                
            # Convert data to numpy arrays for faster processing
            angles = np.array([point[0] for point in polar_data])
            distances = np.array([point[1] for point in polar_data])
            
            # Update the plot
            offsets = np.column_stack((angles, distances))
            line.set_offsets(offsets)
            
            # Color by intensity/distance
            intensity = np.array([0 + (50 - 0) * (d / self.config.LIDAR_DMAX) for d in distances])
            line.set_array(intensity)
            
        return line,

    def update_accel_plot(self, frame, accel_line):
        """Update function for accelerometer animation"""
        with self.accel_data_lock:
            if not self.accel_data:
                return accel_line,
                
            # Update the plot with current data
            data_array = np.array(self.accel_data)
            accel_line.set_ydata(data_array)
            
            # Adjust x-axis for proper scrolling effect
            accel_line.set_xdata(np.arange(len(data_array)))
            
        return accel_line,

    def setup_visualization(self):
        """Set up matplotlib figures and animations"""
        # Set the matplotlib backend properties to allow window management
        # This works across different backends
        plt.rcParams['figure.raise_window'] = False
        
        # User info text for plot titles
        user_info = f"User: {self.config.USER_LOGIN} | Session: {self.config.SYSTEM_START_TIME}"
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # LiDAR visualization
        self.fig_lidar, ax_lidar = plt.subplots(subplot_kw={'polar': True}, figsize=(8, 8))
        self.fig_lidar.canvas.manager.set_window_title('LiDAR Data - SensorFusion')
        
        # Make figure minimizable - using a cross-platform approach
        try:
            # Different backends might have different window management methods
            # Some might use 'window' attribute, others might not
            plt.rcParams['figure.figsize'] = [8, 8]
            plt.rcParams['figure.dpi'] = 100
            
            # Try to detect Qt backend and handle it properly
            if hasattr(self.fig_lidar.canvas.manager, 'window'):
                backend = plt.get_backend().lower()
                if 'qt' in backend:
                    # For Qt backends
                    self.fig_lidar.canvas.manager.window.setWindowFlags(
                        self.fig_lidar.canvas.manager.window.windowFlags() & ~0x00000020
                    )
                elif 'tk' in backend:
                    # For Tk backends 
                    self.fig_lidar.canvas.manager.window.wm_attributes("-topmost", 0)
                # For other backends, we rely on the default behavior
            
            logger.info(f"Configured LiDAR visualization window using backend: {plt.get_backend()}")
        except Exception as e:
            logger.warning(f"Could not configure window manager for LiDAR plot: {e}")
        
        line = ax_lidar.scatter([0, 0], [0, 0], s=5, c=[0, 0], cmap=plt.cm.Greys_r, lw=0)
        
        ax_lidar.set_rmax(1000)  # Set maximum distance to display
        
        # Set the angle limits to show our 90-degree field of view
        ax_lidar.set_thetamin(self.config.LIDAR_MIN_ANGLE)
        ax_lidar.set_thetamax(self.config.LIDAR_MAX_ANGLE)
        
        ax_lidar.grid(True)
        ax_lidar.set_title(f"LiDAR Data (90° FOV: 315°-360° and 0°-45°)\n{user_info}")
        
        self.lidar_ani = animation.FuncAnimation(
            self.fig_lidar, 
            self.update_lidar_plot,
            fargs=(line,), 
            interval=self.config.UPDATE_INTERVAL, 
            blit=True,
            cache_frame_data=False
        )
        
        # Accelerometer visualization
        self.fig_accel, ax_accel = plt.subplots(figsize=(10, 4))
        self.fig_accel.canvas.manager.set_window_title('Accelerometer Data - SensorFusion')
        
        # Make figure minimizable - similar approach for accelerometer plot
        try:
            plt.rcParams['figure.figsize'] = [10, 4]
            
            if hasattr(self.fig_accel.canvas.manager, 'window'):
                backend = plt.get_backend().lower()
                if 'qt' in backend:
                    self.fig_accel.canvas.manager.window.setWindowFlags(
                        self.fig_accel.canvas.manager.window.windowFlags() & ~0x00000020
                    )
                elif 'tk' in backend:
                    self.fig_accel.canvas.manager.window.wm_attributes("-topmost", 0)
            
            logger.info(f"Configured accelerometer visualization window")
        except Exception as e:
            logger.warning(f"Could not configure window manager for accelerometer plot: {e}")
        
        # Initialize with empty data
        accel_line, = ax_accel.plot(
            np.arange(self.config.MAX_DATA_POINTS),
            np.zeros(self.config.MAX_DATA_POINTS),
            'b-', 
            label='Acceleration (Z)'
        )
        
        ax_accel.set_xlim(0, self.config.MAX_DATA_POINTS - 1)
        ax_accel.set_ylim(-2, 2)
        ax_accel.set_title(f"Accelerometer Data\n{user_info}")
        ax_accel.set_xlabel("Sample")
        ax_accel.set_ylabel("Acceleration (g)")
        ax_accel.grid(True)
        ax_accel.legend(loc='upper right')
        
        # Add user info text in the lower right corner
        self.fig_accel.text(0.99, 0.01, f"{user_info} | Current time: {current_time}", 
                           horizontalalignment='right',
                           verticalalignment='bottom',
                           transform=self.fig_accel.transFigure,
                           fontsize=8, alpha=0.7)
        
        self.accel_ani = animation.FuncAnimation(
            self.fig_accel, 
            self.update_accel_plot, 
            fargs=(accel_line,),
            interval=self.config.UPDATE_INTERVAL, 
            blit=True,
            cache_frame_data=False
        )
        
        # Create a basic default map with hardcoded location
        # This ensures we have a map to display even without GPS data
        self.create_default_map()
        
        # Try to open the map in browser
        try:
            map_url = 'file://' + os.path.abspath(self.config.MAP_HTML_PATH)
            logger.info(f"Opening map at: {map_url}")
            if webbrowser.open(map_url):
                logger.info("Map opened in browser")
            else:
                logger.warning("Failed to open browser, but map file was created")
        except Exception as e:
            logger.error(f"Error opening map in browser: {e}")

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
        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Initialize I2C
        if not self.initialize_i2c():
            logger.error("Failed to initialize I2C. Exiting.")
            return
        
        # Initialize sensors
        sensors_ok = True
        
        if not self.initialize_lidar():
            logger.error("Failed to initialize LiDAR. Exiting.")
            sensors_ok = False
            
        if not self.initialize_gps():
            logger.warning("Failed to initialize GPS. Continuing without GPS.")
            
        if not self.initialize_icm20948():
            logger.warning("Failed to initialize ICM20948. Continuing without accelerometer data.")
        
        if not sensors_ok:
            logger.error("Critical sensors failed to initialize. Exiting.")
            self.cleanup()
            return
        
        # Start data collection threads
        self.threads = [
            threading.Thread(target=self.lidar_thread_func, daemon=True),
            threading.Thread(target=self.gps_thread_func, daemon=True),
            threading.Thread(target=self.accel_thread_func, daemon=True)
        ]
        
        for thread in self.threads:
            thread.start()
        
        # Set up and start visualization
        try:
            logger.info("Setting up visualization...")
            self.setup_visualization()
            
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
