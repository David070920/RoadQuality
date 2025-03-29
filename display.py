import serial
import time
import pynmea2
import threading
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from RoadQuality.sensors.lidar_sensor import LidarSensor
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
import sqlite3
import pickle
import json
import cv2
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.cm as cm

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
        SAVE_INTERVAL = 30     # Data save interval in seconds
        
        # User and system information
        USER_LOGIN = "David070920"
        SYSTEM_START_TIME = "2025-03-29 12:31:27"
        
        # LiDAR settings
        LIDAR_PORT = '/dev/ttyUSB0'
        LIDAR_DMAX = 4000      # Maximum LiDAR distance
        LIDAR_SCAN_MODE = 0    # Scan mode (0-2)
        
        # Modified: For perpendicular to ground measurement
        LIDAR_MIN_ANGLE = -45  # Minimum display angle (converted from 315° to -45° for polar plot)
        LIDAR_MAX_ANGLE = 45   # Maximum display angle
        LIDAR_FILTER_ANGLES = [(0, 45), (315, 360)]  # Angles to keep (min, max)
        
        # Road Quality Settings
        ROAD_QUALITY_WINDOW = 20       # Number of samples to analyze for road quality
        MIN_VALID_SAMPLES = 10         # Minimum samples needed for valid measurement
        ROUGHNESS_THRESHOLDS = {       # Thresholds for quality classification
            "excellent": 1.0,
            "good": 3.0,
            "fair": 7.0,
            "poor": 15.0
        }
        
        # GPS settings
        GPS_PORT = '/dev/ttyACM0'
        GPS_BAUD_RATE = 9600
        GPS_TIMEOUT = 0.5
        
        # ICM20948 settings
        ICM20948_ADDRESS = 0x69
        ICM20948_WHO_AM_I = 0x00
        ICM20948_PWR_MGMT_1 = 0x06
        ICM20948_ACCEL_ZOUT_H = 0x31
        
        # Data Storage Settings
        DATA_ROOT_DIR = os.path.join(str(Path.home()), "road_quality_data")
        
        # Folium map settings - Using absolute path in home directory
        MAP_HTML_PATH = os.path.join(str(Path.home()), "road_quality_map.html")
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
        
        # Road quality data structures
        self.road_quality_lock = threading.Lock()
        self.road_quality = {"quality": "Unknown", "quality_score": 50, "roughness": 0, "variance": 0, "distance": 0}
        self.road_quality_history = deque(maxlen=self.config.MAX_DATA_POINTS)
        self.road_profile_history = deque(maxlen=100)  # Last 100 road profiles
        
        # Trip tracking
        self.trip_coords = []
        self.trip_quality = []
        self.last_save_time = time.time()
        
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
        self.fig_quality = None
        self.lidar_ani = None
        self.accel_ani = None
        self.quality_ani = None
        
        # Database and data file paths
        self.trip_id = None
        self.data_dir = None
        self.road_quality_file = None
        self.db_path = None
        
        # Log the map file location
        logger.info(f"GPS map will be saved to: {self.config.MAP_HTML_PATH}")
        
        # Make sure data directory exists
        os.makedirs(self.config.DATA_ROOT_DIR, exist_ok=True)

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

    def setup_recording(self):
        """Set up data recording functionality"""
        try:
            # Create timestamped directory for this trip
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.trip_id = f"trip_{timestamp}"
            self.data_dir = os.path.join(self.config.DATA_ROOT_DIR, self.trip_id)
            os.makedirs(self.data_dir, exist_ok=True)
            
            # Initialize CSV files with headers
            self.road_quality_file = os.path.join(self.data_dir, "road_quality.csv")
            with open(self.road_quality_file, 'w') as f:
                f.write("timestamp,latitude,longitude,quality,quality_score,roughness,variance,distance\n")
            
            # Initialize SQLite database for more structured data
            self.db_path = os.path.join(self.data_dir, "trip_data.db")
            conn = sqlite3.connect(self.db_path)
            c = conn.cursor()
            c.execute('''
                CREATE TABLE IF NOT EXISTS road_quality (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT,
                    latitude REAL,
                    longitude REAL,
                    quality TEXT,
                    quality_score INTEGER,
                    roughness REAL,
                    variance REAL,
                    raw_lidar BLOB
                )
            ''')
            
            # Create table for accelerometer data
            c.execute('''
                CREATE TABLE IF NOT EXISTS accelerometer (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT,
                    accel_x REAL,
                    accel_y REAL,
                    accel_z REAL
                )
            ''')
            
            # Create table for trip metadata
            c.execute('''
                CREATE TABLE IF NOT EXISTS trip_info (
                    key TEXT PRIMARY KEY,
                    value TEXT
                )
            ''')
            
            # Insert trip metadata
            c.execute("INSERT INTO trip_info VALUES (?, ?)", 
                     ("user", self.config.USER_LOGIN))
            c.execute("INSERT INTO trip_info VALUES (?, ?)", 
                     ("start_time", self.config.SYSTEM_START_TIME))
            c.execute("INSERT INTO trip_info VALUES (?, ?)", 
                     ("device_info", json.dumps({
                         "lidar": "RPLidar",
                         "gps": "Generic GPS",
                         "imu": "ICM20948"
                     })))
            
            conn.commit()
            conn.close()
            
            # Create directory for map data
            map_dir = os.path.join(self.data_dir, "maps")
            os.makedirs(map_dir, exist_ok=True)
            
            logger.info(f"Data recording initialized at {self.data_dir}")
            return True
        except Exception as e:
            logger.error(f"Failed to setup recording: {e}")
            return False

    def save_data(self):
        """Save collected data to files"""
        current_time = time.time()
        if current_time - self.last_save_time < self.config.SAVE_INTERVAL:
            return
        
        try:
            # Get current values with locks
            with self.gps_data_lock:
                gps_data = self.gps_data.copy()
                
            with self.road_quality_lock:
                road_quality = self.road_quality.copy()
                
            with self.lidar_data_lock:
                lidar_data = self.lidar_data.copy()
            
            # Save road quality data to CSV
            with open(self.road_quality_file, 'a') as f:
                timestamp = datetime.now().isoformat()
                f.write(f"{timestamp},{gps_data['lat']},{gps_data['lon']},"
                        f"{road_quality['quality']},{road_quality['quality_score']},"
                        f"{road_quality['roughness']},{road_quality['variance']},{road_quality['distance']}\n")
            
            # Save to SQLite database
            conn = sqlite3.connect(self.db_path)
            c = conn.cursor()
            
            # Save road quality with raw LiDAR data
            c.execute('''
                INSERT INTO road_quality 
                (timestamp, latitude, longitude, quality, quality_score, roughness, variance, raw_lidar)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                datetime.now().isoformat(),
                gps_data['lat'],
                gps_data['lon'],
                road_quality['quality'],
                road_quality['quality_score'],
                road_quality['roughness'],
                road_quality['variance'],
                pickle.dumps(lidar_data)  # Store raw LiDAR data for later analysis
            ))
            
            # Save latest accelerometer data
            with self.accel_data_lock:
                if self.accel_data:
                    accel_z = self.accel_data[-1]
                    c.execute('''
                        INSERT INTO accelerometer (timestamp, accel_x, accel_y, accel_z)
                        VALUES (?, ?, ?, ?)
                    ''', (
                        datetime.now().isoformat(),
                        0.0,  # We're not collecting X/Y in this version
                        0.0,  # We're not collecting X/Y in this version
                        accel_z
                    ))
            
            conn.commit()
            conn.close()
            
            # Save a snapshot of the map
            map_snapshot_path = os.path.join(self.data_dir, "maps", f"map_{int(current_time)}.html")
            if os.path.exists(self.config.MAP_HTML_PATH):
                import shutil
                shutil.copy2(self.config.MAP_HTML_PATH, map_snapshot_path)
            
            self.last_save_time = current_time
            logger.debug(f"Data saved at {datetime.now().isoformat()}")
            
        except Exception as e:
            logger.error(f"Error saving data: {e}")

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

    def measure_road_quality(self, lidar_scan):
        """Analyze LiDAR data to determine road quality metrics"""
        # Extract ground profile from perpendicular LiDAR
        angles = []
        distances = []
        
        for point in lidar_scan:
            angle_deg = point[0]
            distance = point[1]
            
            # Convert 315-360 degrees to -45-0 degrees for consistency
            if angle_deg >= 315 and angle_deg <= 360:
                angle_deg = angle_deg - 360
                
            # Only use angles in our target range (-45 to 45 degrees)
            if -45 <= angle_deg <= 45:
                angles.append(angle_deg)
                distances.append(distance)
        
        # Convert to numpy arrays for faster processing
        angles = np.array(angles)
        distances = np.array(distances)
        
        # Filter out invalid measurements (too close or too far)
        valid_mask = (distances > 50) & (distances < 1000)
        valid_angles = angles[valid_mask]
        valid_distances = distances[valid_mask]
        
        if len(valid_distances) < self.config.MIN_VALID_SAMPLES:
            return {
                "quality": "Unknown",
                "quality_score": 50,  # Default medium quality
                "roughness": 0,
                "variance": 0,
                "distance": 0
            }
        
        # Calculate metrics
        mean_distance = np.mean(valid_distances)
        variance = np.var(valid_distances)
        
        # Create a normalized road profile (actual ground height variation)
        # This requires sorting by angle for proper display
        idx = np.argsort(valid_angles)
        sorted_angles = valid_angles[idx]
        sorted_distances = valid_distances[idx]
        
        # Normalize distances around the mean (convert to height variations in mm)
        height_variations = (sorted_distances - mean_distance) * -1.0  # Invert so bumps go up
        
        # Add to road profile history (for visualization)
        self.road_profile_history.append(height_variations)
        
        # Calculate roughness index (normalized by distance to make it scale-invariant)
        roughness_index = variance / mean_distance * 100
        
        # Classify road quality based on thresholds
        thresholds = self.config.ROUGHNESS_THRESHOLDS
        if roughness_index < thresholds["excellent"]:
            quality = "Excellent"
            quality_score = 90
        elif roughness_index < thresholds["good"]:
            quality = "Good"
            quality_score = 75
        elif roughness_index < thresholds["fair"]:
            quality = "Fair"
            quality_score = 50
        elif roughness_index < thresholds["poor"]:
            quality = "Poor"
            quality_score = 25
        else:
            quality = "Very Poor"
            quality_score = 10
        
        return {
            "quality": quality,
            "quality_score": quality_score,
            "roughness": roughness_index,
            "variance": variance,
            "distance": mean_distance,
            "profile": height_variations
        }

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
                
                # Process for road quality analysis
                road_quality = self.measure_road_quality(filtered_data)
                
                # Update road quality data with lock
                with self.road_quality_lock:
                    self.road_quality = road_quality
                    self.road_quality_history.append(road_quality["quality_score"])
                    
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
                        self.update_road_quality_map()
                        self.last_map_update = current_time
                        
                        # Also save data periodically
                        self.save_data()
                    
                    logger.info(f"GPS: {self.gps_data}")
                    
            except Exception as e:
                logger.debug(f"Error in GPS thread: {e}")
                
            # Sleep to prevent high CPU usage
            time.sleep(0.2)
            
        logger.info("GPS thread stopped")

    def update_road_quality_map(self):
        """Update map with color-coded road segments based on quality"""
        try:
            with self.gps_data_lock:
                lat = self.gps_data["lat"]
                lon = self.gps_data["lon"]
                alt = self.gps_data["alt"]
                sats = self.gps_data["sats"]
                timestamp = self.gps_data["timestamp"]
            
            with self.road_quality_lock:
                quality = self.road_quality["quality"]
                quality_score = self.road_quality["quality_score"]
                roughness = self.road_quality["roughness"]
            
            # Skip if we don't have valid coordinates yet
            if lat == 0 and lon == 0:
                logger.warning("No valid GPS coordinates yet, skipping map update")
                return
                
            # Append to path for this trip
            self.trip_coords.append((lat, lon))
            self.trip_quality.append(quality_score)
            
            # Create a map centered at the current location
            m = folium.Map(location=[lat, lon], zoom_start=self.config.MAP_ZOOM_START)
            
            # Add color-coded line segments based on quality
            if len(self.trip_coords) > 1:
                for i in range(len(self.trip_coords) - 1):
                    # Determine color based on road quality
                    if self.trip_quality[i] >= 80:
                        color = 'green'
                    elif self.trip_quality[i] >= 60:
                        color = 'lightgreen'
                    elif self.trip_quality[i] >= 40:
                        color = 'yellow'
                    elif self.trip_quality[i] >= 20:
                        color = 'orange'
                    else:
                        color = 'red'
                        
                    # Add this segment to the map
                    folium.PolyLine(
                        [self.trip_coords[i], self.trip_coords[i + 1]],
                        color=color,
                        weight=5,
                        opacity=0.8,
                        tooltip=f"Quality: {self.trip_quality[i]}/100"
                    ).add_to(m)
            
            # Add current position marker
            popup_text = f"""
            <b>Road Quality: {quality}</b><br>
            Quality Score: {quality_score}/100<br>
            Roughness Index: {roughness:.2f}<br>
            <hr>
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
                popup=folium.Popup(popup_text, max_width=300),
                tooltip=f"Quality: {quality} ({quality_score}/100)",
                icon=folium.Icon(color='blue')
            ).add_to(m)
            
            # Add a circle to show accuracy (just for visualization)
            folium.Circle(
                location=[lat, lon],
                radius=10,  # 10 meters radius
                color='blue',
                fill=True,
                fill_opacity=0.2
            ).add_to(m)
            
            # Add a legend for road quality
            legend_html = '''
            <div style="position: fixed; 
                 bottom: 50px; left: 50px; width: 150px; height: 160px; 
                 border:2px solid grey; z-index:9999; font-size:12px;
                 background-color:white; padding: 10px;
                 border-radius: 5px;">
                <p style="margin-top: 0; font-weight: bold;">Road Quality</p>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: green; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Excellent (80-100)</span>
                </div>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: lightgreen; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Good (60-80)</span>
                </div>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: yellow; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Fair (40-60)</span>
                </div>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: orange; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Poor (20-40)</span>
                </div>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: red; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Very Poor (0-20)</span>
                </div>
            </div>
            '''
            m.get_root().html.add_child(folium.Element(legend_html))
            
            # Save the map to an HTML file
            m.save(self.config.MAP_HTML_PATH)
            logger.info(f"Road quality map updated at {self.config.MAP_HTML_PATH}")
            
        except Exception as e:
            logger.error(f"Error updating road quality map: {e}")

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
            
            # Add legend for road quality
            legend_html = '''
            <div style="position: fixed; 
                 bottom: 50px; left: 50px; width: 150px; height: 160px; 
                 border:2px solid grey; z-index:9999; font-size:12px;
                 background-color:white; padding: 10px;
                 border-radius: 5px;">
                <p style="margin-top: 0; font-weight: bold;">Road Quality</p>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: green; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Excellent (80-100)</span>
                </div>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: lightgreen; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Good (60-80)</span>
                </div>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: yellow; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Fair (40-60)</span>
                </div>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: orange; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Poor (20-40)</span>
                </div>
                <div style="display: flex; align-items: center; margin-bottom: 5px;">
                    <div style="background-color: red; width: 20px; height: 10px; margin-right: 5px;"></div>
                    <span>Very Poor (0-20)</span>
                </div>
            </div>
            '''
            m.get_root().html.add_child(folium.Element(legend_html))
            
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

    def update_quality_plot(self, frame):
        """Update function for road quality visualization"""
        with self.road_quality_lock:
            # Update quality score history plot
            if self.road_quality_history:
                quality_data = np.array(self.road_quality_history)
                x_data = np.arange(len(quality_data))
                self.quality_line.set_data(x_data, quality_data)
                
                # Adjust x-axis limits for scrolling effect
                self.ax_quality.set_xlim(max(0, len(quality_data) - 50), max(50, len(quality_data)))
                
                                # Update quality score text
                current_quality = self.road_quality["quality"]
                current_score = self.road_quality["quality_score"]
                self.quality_text.set_text(f"Current Quality: {current_quality} ({current_score}/100)")
                
                # Update quality indicator color
                if current_score >= 80:
                    color = 'green'
                elif current_score >= 60:
                    color = 'lightgreen'
                elif current_score >= 40:
                    color = 'yellow'
                elif current_score >= 20:
                    color = 'orange'
                else:
                    color = 'red'
                self.quality_indicator.set_color(color)
            
            # Update road profile visualization
            if self.road_profile_history:
                # Create a 2D array for the profile history
                profiles = np.zeros((min(50, len(self.road_profile_history)), 100))
                
                # Fill in available profiles
                for i, profile in enumerate(list(self.road_profile_history)[-50:]):
                    # Ensure profile has enough points, resize if needed
                    if len(profile) > 0:
                        # Interpolate to 100 points if necessary
                        if len(profile) != 100:
                            indices = np.linspace(0, len(profile)-1, 100).astype(int)
                            profile = profile[indices]
                        
                        # Clip extreme values
                        profile = np.clip(profile, -25, 25)
                        
                        # Add to the profiles array
                        profiles[i % 50] = profile
                
                # Update the road profile image
                self.road_profile.set_array(profiles)
                
                # Update colorbar if it exists
                if hasattr(self, 'road_profile_cbar'):
                    self.road_profile_cbar.update_normal(self.road_profile)
        
        # Return all artists that were updated
        return self.quality_line, self.quality_text, self.quality_indicator, self.road_profile

    def detect_road_anomalies(self, lidar_data):
        """Detect specific road anomalies like potholes and bumps"""
        # Extract perpendicular distance measurements (ground points)
        ground_points = []
        for point in lidar_data:
            angle, distance = point
            angle_norm = angle % 360
            # Consider only points aimed close to perpendicular to ground
            if 355 <= angle_norm <= 359 or 0 <= angle_norm <= 5:
                ground_points.append(distance)
        
        if len(ground_points) < 10:  # Need minimum points for analysis
            return {"anomalies": [], "has_anomaly": False}
        
        # Calculate rolling median as the baseline
        ground_points = np.array(ground_points)
        median_baseline = np.median(ground_points)
        
        # Calculate deviation from baseline
        deviations = abs(ground_points - median_baseline)
        
        # Define thresholds for anomalies (in mm)
        POTHOLE_THRESHOLD = 30    # 3cm deeper than surroundings
        BUMP_THRESHOLD = 20       # 2cm higher than surroundings
        CRACK_THRESHOLD = 15      # 1.5cm variation
        
        # Detect anomalies
        anomalies = []
        
        # Look for potholes (significantly farther distance)
        pothole_indices = np.where(ground_points - median_baseline > POTHOLE_THRESHOLD)[0]
        if len(pothole_indices) > 0:
            clusters = self._cluster_anomalies(pothole_indices)
            for cluster in clusters:
                if len(cluster) >= 2:  # Require at least 2 points to confirm pothole
                    depth = np.max(ground_points[cluster] - median_baseline)
                    width = len(cluster) * 2  # Approximate width in cm
                    anomalies.append({
                        "type": "pothole",
                        "depth_mm": float(depth),
                        "width_cm": width,
                        "severity": "high" if depth > 50 else "medium",
                        "indices": cluster.tolist()
                    })
        
        # Look for bumps (significantly closer distance)
        bump_indices = np.where(median_baseline - ground_points > BUMP_THRESHOLD)[0]
        if len(bump_indices) > 0:
            clusters = self._cluster_anomalies(bump_indices)
            for cluster in clusters:
                if len(cluster) >= 2:  # Require at least 2 points to confirm bump
                    height = np.max(median_baseline - ground_points[cluster])
                    width = len(cluster) * 2  # Approximate width in cm
                    anomalies.append({
                        "type": "bump",
                        "height_mm": float(height),
                        "width_cm": width,
                        "severity": "high" if height > 30 else "medium",
                        "indices": cluster.tolist()
                    })
        
        # Look for cracks (rapid variations)
        if len(ground_points) > 3:
            diffs = np.abs(np.diff(ground_points))
            crack_indices = np.where(diffs > CRACK_THRESHOLD)[0]
            if len(crack_indices) > 0:
                clusters = self._cluster_anomalies(crack_indices)
                for cluster in clusters:
                    if len(cluster) >= 1:  # Even single points can indicate cracks
                        anomalies.append({
                            "type": "crack",
                            "width_mm": float(np.mean(diffs[cluster])),
                            "severity": "medium",
                            "indices": cluster.tolist()
                        })
        
        # Return all detected anomalies
        return {
            "anomalies": anomalies,
            "has_anomaly": len(anomalies) > 0,
            "baseline_mm": float(median_baseline)
        }

    def _cluster_anomalies(self, indices, max_gap=2):
        """Group adjacent anomaly indices into clusters"""
        indices = np.sort(indices)
        if len(indices) == 0:
            return []
        
        clusters = []
        current_cluster = [indices[0]]
        
        for i in range(1, len(indices)):
            if indices[i] - indices[i-1] <= max_gap:
                current_cluster.append(indices[i])
            else:
                clusters.append(np.array(current_cluster))
                current_cluster = [indices[i]]
        
        clusters.append(np.array(current_cluster))
        return clusters

    def alert_user_about_hazards(self, anomalies):
        """Alert the user about detected road hazards"""
        # Skip if no anomalies are detected
        if not anomalies.get("has_anomaly", False):
            return
            
        try:
            anomaly_list = anomalies.get("anomalies", [])
            
            # Log the anomalies
            for anomaly in anomaly_list:
                severity = anomaly.get("severity", "medium")
                a_type = anomaly.get("type", "unknown")
                
                # Different log level based on severity
                if severity == "high":
                    logger.warning(f"HIGH SEVERITY {a_type.upper()} DETECTED!")
                else:
                    logger.info(f"{severity.capitalize()} {a_type} detected")
                
                # Update plot title with warning if anomaly is severe
                if severity == "high" and hasattr(self, 'fig_quality'):
                    self.fig_quality.suptitle(f"⚠️ CAUTION: {a_type.upper()} AHEAD ⚠️", 
                                             color='red', fontsize=16)
                    # Reset title after 3 seconds
                    threading.Timer(3.0, self.reset_plot_title).start()
            
            # Save anomaly data to database
            if self.db_path:
                try:
                    conn = sqlite3.connect(self.db_path)
                    c = conn.cursor()
                    
                    # Check if anomalies table exists, create if not
                    c.execute('''CREATE TABLE IF NOT EXISTS anomalies
                              (id INTEGER PRIMARY KEY AUTOINCREMENT,
                               timestamp TEXT,
                               latitude REAL,
                               longitude REAL,
                               type TEXT,
                               severity TEXT,
                               details TEXT)''')
                    
                    # Get current GPS position
                    with self.gps_data_lock:
                        lat = self.gps_data.get("lat", 0)
                        lon = self.gps_data.get("lon", 0)
                    
                    # Insert each anomaly
                    for anomaly in anomaly_list:
                        c.execute('''INSERT INTO anomalies
                                  (timestamp, latitude, longitude, type, severity, details)
                                  VALUES (?, ?, ?, ?, ?, ?)''',
                                  (datetime.now().isoformat(),
                                   lat, lon,
                                   anomaly.get("type", "unknown"),
                                   anomaly.get("severity", "medium"),
                                   json.dumps(anomaly)))
                    
                    conn.commit()
                    conn.close()
                    
                except Exception as e:
                    logger.error(f"Error saving anomaly data: {e}")
            
        except Exception as e:
            logger.error(f"Error in hazard alert: {e}")

    def reset_plot_title(self):
        """Reset the plot title after an alert"""
        if hasattr(self, 'fig_quality'):
            self.fig_quality.suptitle("Road Quality Analysis", fontsize=14)

    def setup_battery_monitoring(self):
        """Set up battery monitoring"""
        # Default battery state
        self.battery_status = {
            "percentage": 100,
            "voltage": 5.0,
            "charging": False,
            "estimated_runtime": 180,  # minutes
            "timestamp": datetime.now().isoformat()
        }
        
        # If your hardware supports battery monitoring, implement here
        try:
            # Check if the platform is a Raspberry Pi with battery monitoring
            if os.path.exists("/sys/class/power_supply/battery"):
                self.has_battery_monitoring = True
                # Initialize the battery monitoring thread
                self.battery_thread = threading.Thread(
                    target=self.battery_monitoring_thread,
                    daemon=True
                )
                self.battery_thread.start()
                logger.info("Battery monitoring initialized")
            else:
                logger.warning("No hardware battery monitoring available")
                self.has_battery_monitoring = False
                
            return True
        except Exception as e:
            logger.error(f"Error setting up battery monitoring: {e}")
            self.has_battery_monitoring = False
            return False

    def battery_monitoring_thread(self):
        """Thread for monitoring battery status"""
        logger.info("Battery monitoring thread started")
        self.low_battery_warning_shown = False
        
        while not self.stop_event.is_set():
            try:
                # Simulate battery drainage for testing or implement actual reading
                # This is just an example - replace with actual battery monitoring code
                if hasattr(self, 'battery_status'):
                    # Decrease battery by 0.1% every 30 seconds (for simulation)
                    now = datetime.now()
                    time_diff = (now - datetime.fromisoformat(self.battery_status["timestamp"])).total_seconds()
                    
                    if time_diff > 30:
                        new_percentage = max(0, self.battery_status["percentage"] - 0.1)
                        self.battery_status = {
                            "percentage": new_percentage,
                            "voltage": 3.7 + (new_percentage / 100 * 0.8),  # Simulate voltage between 3.7-4.5V
                            "charging": False,
                            "estimated_runtime": int(new_percentage * 1.8),  # 3 hours from 100%
                            "timestamp": now.isoformat()
                        }
                    
                    # Check if battery is critically low
                    if self.battery_status["percentage"] < 10 and not self.low_battery_warning_shown:
                        logger.warning("BATTERY LOW - ONLY 10% REMAINING")
                        self.show_low_battery_warning()
                        self.low_battery_warning_shown = True
                    
                    # Reset warning flag if battery charged above 15%
                    if self.battery_status["percentage"] > 15 and self.low_battery_warning_shown:
                        self.low_battery_warning_shown = False
            except Exception as e:
                logger.error(f"Error in battery monitoring: {e}")
            
            time.sleep(5)  # Check every 5 seconds
        
        logger.info("Battery monitoring thread stopped")

    def get_battery_status(self):
        """Get current battery status"""
        if hasattr(self, 'battery_status'):
            return self.battery_status
        else:
            return {
                "percentage": "Unknown",
                "voltage": "Unknown",
                "charging": False,
                "estimated_runtime": "Unknown"
            }

    def show_low_battery_warning(self):
        """Display low battery warning"""
        try:
            # Show a message on the visualization
            if self.fig_quality:
                self.fig_quality.suptitle("⚠️ LOW BATTERY - SAVE YOUR DATA ⚠️", color='red', fontsize=16)
            
            # You can also implement other warning methods here:
            # - LED blinking
            # - Audio alert
            # - Automatic data saving
            
            # Start power saving mode
            self.enable_power_saving_mode()
        except Exception as e:
            logger.error(f"Error showing battery warning: {e}")

    def enable_power_saving_mode(self):
        """Enable power saving mode to extend battery life"""
        logger.info("Enabling power saving mode")
        
        # Reduce LiDAR scan frequency if possible
        try:
            if self.lidar_device:
                # Change scan mode to lowest power consumption
                self.lidar_device.stopmotor()
                time.sleep(0.5)
                self.lidar_device.startmotor(my_scanmode=2)  # Lower power mode
                logger.info("Reduced LiDAR power consumption")
        except Exception as e:
            logger.error(f"Error adjusting LiDAR power: {e}")
        
        # Reduce data processing frequency
        self.config.UPDATE_INTERVAL = 500  # ms
        
        # Update user about power saving mode
        logger.info("Power saving mode activated - reduced functionality to extend battery life")

    def calibrate_sensors(self):
        """Perform sensor calibration routine at startup"""
        logger.info("Starting sensor calibration...")
        
        # Step 1: Calibrate accelerometer to detect orientation
        accel_samples = []
        for _ in range(50):  # Collect 50 samples
            accel_z = self.get_accel_data()
            if accel_z is not None:
                accel_samples.append(accel_z)
            time.sleep(0.01)
        
        if accel_samples:
            # Calculate baseline gravitational acceleration
            self.accel_baseline = np.mean(accel_samples)
            logger.info(f"Accelerometer baseline: {self.accel_baseline:.3f}g")
            
            # Detect orientation based on baseline
            if 0.8 < self.accel_baseline < 1.2:
                self.orientation = "horizontal"
            elif -0.2 < self.accel_baseline < 0.2:
                self.orientation = "vertical"
            else:
                self.orientation = "tilted"
                
            logger.info(f"Detected orientation: {self.orientation}")
        else:
            logger.warning("Could not calibrate accelerometer")
            self.accel_baseline = 1.0
            self.orientation = "unknown"
        
        # Step 2: Calibrate LiDAR distance to ground (stationary)
        lidar_samples = []
        logger.info("Calibrating LiDAR ground distance. Keep the bike stationary...")
        
        for _ in range(10):  # Collect 10 scans
            try:
                scan_data = self.lidar_device.get_scan_as_vectors(filter_quality=True)
                filtered_data = self.filter_lidar_angles(scan_data)
                
                # Get the median distance for the points directly below
                distances = [point[1] for point in filtered_data if -5 <= (point[0] % 360) <= 5]
                if distances:
                    lidar_samples.append(np.median(distances))
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Error during LiDAR calibration: {e}")
        
        if lidar_samples:
            # Set the ground distance baseline
            self.ground_distance_baseline = np.median(lidar_samples)
            logger.info(f"Ground distance baseline: {self.ground_distance_baseline:.2f}mm")
        else:
            logger.warning("Could not calibrate LiDAR ground distance")
            self.ground_distance_baseline = 300.0  # Default 30cm assumption
        
        # Store calibration data
        self.calibration_data = {
            "timestamp": datetime.now().isoformat(),
            "accel_baseline": float(self.accel_baseline),
            "ground_distance": float(self.ground_distance_baseline),
            "orientation": self.orientation
        }
        
        # Save calibration data
        try:
            with open(os.path.join(self.data_dir, "calibration.json"), 'w') as f:
                json.dump(self.calibration_data, f, indent=2)
        except Exception as e:
            logger.error(f"Error saving calibration data: {e}")
        
        logger.info("Sensor calibration completed")
        return True

    def apply_motion_compensation(self, lidar_data, accel_data):
        """Apply motion compensation to LiDAR readings based on accelerometer data"""
        # Skip if no accelerometer data is available
        if not accel_data:
            return lidar_data
        
        # Calculate vertical displacement from accelerometer
        # Using simplified physics model: d = 0.5 * a * t²
        # Where a is acceleration in m/s² and t is time between samples
        current_accel = accel_data[-1] - self.accel_baseline  # Remove gravity component
        time_delta = 0.1  # Approximate time between samples (100ms)
        
        # Convert acceleration to m/s² (from g)
        accel_ms2 = current_accel * 9.81
        
        # Calculate displacement in mm
        displacement_mm = 0.5 * accel_ms2 * (time_delta ** 2) * 1000
        
        # Apply displacement compensation to each LiDAR reading
        compensated_data = []
        for point in lidar_data:
            angle, distance = point
            
            # Only compensate points that are measuring the ground
            # (approximately -5° to 5° from vertical)
            angle_normalized = angle % 360
            if 0 <= angle_normalized <= 10 or 350 <= angle_normalized <= 359:
                # Adjust distance by calculated displacement
                adjusted_distance = distance - displacement_mm
                compensated_data.append((angle, adjusted_distance))
            else:
                # Keep other angles unchanged
                compensated_data.append(point)
        
        return compensated_data

    def setup_road_quality_visualization(self):
        """Set up visualization specifically for road quality"""
        # Create a figure for road quality metrics
        self.fig_quality, (self.ax_quality, self.ax_profile) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig_quality.canvas.manager.set_window_title('Road Quality Metrics')
        self.fig_quality.suptitle("Road Quality Analysis", fontsize=14)
        
        # Top plot: Road quality score over time (0-100)
        self.quality_line, = self.ax_quality.plot([], [], 'g-', linewidth=2)
        self.ax_quality.set_ylim(0, 100)
        self.ax_quality.set_title("Road Quality Score")
        self.ax_quality.set_ylabel("Quality (0-100)")
        self.ax_quality.grid(True)
        
        # Add horizontal regions for quality categories
        self.ax_quality.axhspan(80, 100, alpha=0.2, color='green', label='Excellent')
        self.ax_quality.axhspan(60, 80, alpha=0.2, color='lightgreen', label='Good')
        self.ax_quality.axhspan(40, 60, alpha=0.2, color='yellow', label='Fair')
        self.ax_quality.axhspan(20, 40, alpha=0.2, color='orange', label='Poor')
        self.ax_quality.axhspan(0, 20, alpha=0.2, color='red', label='Very Poor')
        self.ax_quality.legend(loc='upper right')
        
        # Add quality indicator and text
        self.quality_text = self.ax_quality.text(0.02, 0.92, "Current Quality: Unknown", 
                                               transform=self.ax_quality.transAxes,
                                               fontsize=12, weight='bold')
        
        self.quality_indicator = self.ax_quality.scatter([0], [50], s=100, c='yellow', 
                                                       edgecolor='black', zorder=5)
        
        # Bottom plot: Road profile from LiDAR (heatmap showing height variations)
        # Create custom colormap for road surface
        colors = [(0.6, 0, 0), (1, 0, 0), (1, 1, 0), (0, 0.8, 0), (0, 0, 1)]  # R -> G -> B
        positions = [0, 0.25, 0.5, 0.75, 1]
        road_cmap = LinearSegmentedColormap.from_list("RoadQuality", list(zip(positions, colors)))
        
        # Create heatmap for road profile
        self.road_profile = self.ax_profile.imshow(
            np.zeros((50, 100)),  # Empty placeholder
            aspect='auto',
            cmap=road_cmap,
            vmin=-25,  # 25mm below baseline
            vmax=25,   # 25mm above baseline
            origin='lower',
            extent=[0, 100, 0, 50],  # X samples, Y time (latest at bottom)
            interpolation='nearest'
        )
        
        # Add colorbar
        self.road_profile_cbar = plt.colorbar(self.road_profile, ax=self.ax_profile)
        self.road_profile_cbar.set_label('Height Variation (mm)')
        
        self.ax_profile.set_title("Road Surface Profile (Latest 50 Scans)")
        self.ax_profile.set_xlabel("Sample Points")
        self.ax_profile.set_ylabel("Time (Latest at Top)")
        
        # Flip the y-axis so newest data is at the top
        self.ax_profile.invert_yaxis()
        
        self.fig_quality.tight_layout(rect=[0, 0, 1, 0.95])  # Leave room for suptitle
        
        # Create animation
        self.quality_ani = animation.FuncAnimation(
            self.fig_quality,
            self.update_quality_plot,
            interval=self.config.UPDATE_INTERVAL,
            blit=True,
            cache_frame_data=False
        )

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
        
        # Configure window appearance
        try:
            plt.rcParams['figure.figsize'] = [8, 8]
            plt.rcParams['figure.dpi'] = 100
            
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
            
            logger.info(f"Configured LiDAR visualization window using backend: {plt.get_backend()}")
        except Exception as e:
            logger.warning(f"Could not configure window manager for LiDAR plot: {e}")
        
        line = ax_lidar.scatter([0, 0], [0, 0], s=5, c=[0, 0], cmap=plt.cm.Greys_r, lw=0)
        
        ax_lidar.set_rmax(1000)  # Set maximum distance to display
        
        # Set the angle limits to show our 90-degree field of view
        ax_lidar.set_thetamin(self.config.LIDAR_MIN_ANGLE)
        ax_lidar.set_thetamax(self.config.LIDAR_MAX_ANGLE)
        
        ax_lidar.grid(True)
        ax_lidar.set_title(f"LiDAR Data (Perpendicular to Ground)\n{user_info}")
        
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
        
        # Configure window appearance
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
        ax_accel.set_title(f"Accelerometer Data (Z-axis)\n{user_info}")
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
        
        # Setup the road quality visualization window
        self.setup_road_quality_visualization()
        
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

    def export_trip_data(self, format_type="all"):
    """Export trip data in various formats"""
    if not hasattr(self, 'trip_id') or not self.trip_id:
        logger.error("No trip data to export")
        return False
    
    export_dir = os.path.join(self.data_dir, "exports")
    os.makedirs(export_dir, exist_ok=True)
    
    # Base filename using trip ID
    base_filename = f"road_quality_{self.trip_id}"
    
    # Create export summary
    export_results = {"success": False, "files": []}
    
    try:
        # Export to CSV
        if format_type in ["csv", "all"]:
            csv_path = os.path.join(export_dir, f"{base_filename}.csv")
            
            # Get data from database
            conn = sqlite3.connect(self.db_path)
            df = pd.read_sql_query("SELECT * FROM road_quality", conn)
            conn.close()
            
            # Save to CSV
            df.to_csv(csv_path, index=False)
            export_results["files"].append(csv_path)
            logger.info(f"Exported CSV data to {csv_path}")
        
        # Export to GeoJSON
        if format_type in ["geojson", "all"]:
            geojson_path = os.path.join(export_dir, f"{base_filename}.geojson")
            
            # Get data from database
            conn = sqlite3.connect(self.db_path)
            df = pd.read_sql_query(
                "SELECT timestamp, latitude, longitude, quality, quality_score, roughness FROM road_quality", 
                conn
            )
            conn.close()
            
            # Create GeoJSON features
            features = []
            for _, row in df.iterrows():
                if row['latitude'] and row['longitude']:  # Only include points with valid coordinates
                    feature = {
                        "type": "Feature",
                        "geometry": {
                            "type": "Point",
                            "coordinates": [row['longitude'], row['latitude']]
                        },
                        "properties": {
                            "timestamp": row['timestamp'],
                            "quality": row['quality'],
                            "quality_score": int(row['quality_score']),
                            "roughness": float(row['roughness'])
                        }
                    }
                    features.append(feature)
            
            # Create GeoJSON structure
            geojson = {
                "type": "FeatureCollection",
                "name": f"Road Quality - {self.trip_id}",
                "features": features
            }
            
            # Save to file
            with open(geojson_path, 'w') as f:
                json.dump(geojson, f, indent=2)
            
            export_results["files"].append(geojson_path)
            logger.info(f"Exported GeoJSON data to {geojson_path}")
            
        # Export to HTML report
        if format_type in ["html", "all"]:
            html_path = os.path.join(export_dir, f"{base_filename}_report.html")
            
            # Get data from database
            conn = sqlite3.connect(self.db_path)
            
            # Summary statistics
            c = conn.cursor()
            c.execute("SELECT COUNT(*) FROM road_quality")
            total_points = c.fetchone()[0]
            
            c.execute("SELECT AVG(quality_score) FROM road_quality")
            avg_quality = c.fetchone()[0] or 0
            
            c.execute("SELECT MIN(timestamp), MAX(timestamp) FROM road_quality")
            times = c.fetchone()
            if times[0] and times[1]:
                start_time = datetime.fromisoformat(times[0])
                end_time = datetime.fromisoformat(times[1])
                trip_duration = (end_time - start_time).total_seconds() / 60  # in minutes
            else:
                trip_duration = 0
            
            # Quality distribution
            c.execute("""
                SELECT quality, COUNT(*) as count 
                FROM road_quality 
                GROUP BY quality 
                ORDER BY CASE 
                    WHEN quality = 'Excellent' THEN 1 
                    WHEN quality = 'Good' THEN 2 
                    WHEN quality = 'Fair' THEN 3 
                    WHEN quality = 'Poor' THEN 4 
                    WHEN quality = 'Very Poor' THEN 5 
                    ELSE 6 
                END
            """)
            quality_dist = c.fetchall()
            
            # Get anomaly statistics
            c.execute("""
                SELECT 
                    SUM(CASE WHEN json_extract(anomalies, '$.has_anomaly') = 1 THEN 1 ELSE 0 END) as total_anomalies,
                    COUNT(DISTINCT latitude || ',' || longitude) as unique_locations
                FROM road_quality
            """)
            anomaly_stats = c.fetchone()
            total_anomalies = anomaly_stats[0] or 0
            unique_locations = anomaly_stats[1] or 0
            
            conn.close()
            
            # Create HTML content
            html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Road Quality Report - {self.trip_id}</title>
    <style>
        body {{ font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; color: #333; }}
        .container {{ max-width: 1200px; margin: 0 auto; }}
        header {{ background-color: #2c3e50; color: white; padding: 20px; border-radius: 5px; margin-bottom: 20px; }}
        h1, h2, h3 {{ margin-top: 0; }}
        .summary-card {{ background-color: #f8f9fa; border-radius: 5px; padding: 15px; margin-bottom: 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        .stat-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-top: 15px; }}
        .stat-box {{ background-color: white; padding: 15px; border-radius: 5px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); text-align: center; }}
        .stat-value {{ font-size: 24px; font-weight: bold; margin: 10px 0; }}
        .quality-chart {{ display: flex; height: 200px; align-items: flex-end; margin: 20px 0; }}
        .quality-bar {{ flex: 1; margin: 0 5px; position: relative; }}
        .bar-fill {{ width: 100%; bottom: 0; position: absolute; transition: height 0.3s; }}
        .excellent {{ background-color: #28a745; }}
        .good {{ background-color: #5cb85c; }}
        .fair {{ background-color: #ffc107; }}
        .poor {{ background-color: #fd7e14; }}
        .very-poor {{ background-color: #dc3545; }}
        .bar-label {{ position: absolute; top: -25px; width: 100%; text-align: center; font-size: 12px; }}
        .percent-label {{ position: absolute; width: 100%; text-align: center; top: -45px; font-weight: bold; }}
        footer {{ margin-top: 30px; text-align: center; color: #666; font-size: 12px; }}
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>Road Quality Analysis Report</h1>
            <p>Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} by SensorFusion</p>
        </header>
        
        <section class="summary-card">
            <h2>Trip Summary</h2>
            <div class="stat-grid">
                <div class="stat-box">
                    <div>Trip Duration</div>
                    <div class="stat-value">{trip_duration:.1f} min</div>
                </div>
                <div class="stat-box">
                    <div>Data Points</div>
                    <div class="stat-value">{total_points}</div>
                </div>
                <div class="stat-box">
                    <div>Average Quality</div>
                    <div class="stat-value">{avg_quality:.1f}/100</div>
                </div>
                <div class="stat-box">
                    <div>Anomalies Detected</div>
                    <div class="stat-value">{total_anomalies}</div>
                </div>
            </div>
        </section>
        
        <section class="summary-card">
            <h2>Road Quality Distribution</h2>
            <div class="quality-chart">
"""
            
            # Add quality distribution bars
            for quality, count in quality_dist:
                percentage = (count / total_points) * 100 if total_points > 0 else 0
                css_class = quality.lower().replace(' ', '-')
                
                html_content += f"""
                <div class="quality-bar">
                    <div class="percent-label">{percentage:.1f}%</div>
                    <div class="bar-label">{quality}</div>
                    <div class="bar-fill {css_class}" style="height: {percentage}%;"></div>
                </div>"""
            
            html_content += """
            </div>
        </section>
        
        <section class="summary-card">
            <h2>Trip Information</h2>
            <p><strong>Trip ID:</strong> """ + self.trip_id + """</p>
            <p><strong>User:</strong> """ + self.config.USER_LOGIN + """</p>
            <p><strong>System Start Time:</strong> """ + self.config.SYSTEM_START_TIME + """</p>
            <p><strong>Measured Road Segments:</strong> """ + str(unique_locations) + """</p>
        </section>
        
        <footer>
            <p>Generated by RoadQuality Sensor Fusion System &copy; 2025</p>
        </footer>
    </div>
</body>
</html>
"""
            
            # Write HTML file
            with open(html_path, 'w') as f:
                f.write(html_content)
                
            export_results["files"].append(html_path)
            logger.info(f"Exported HTML report to {html_path}")
            
        # Generate GPX file for GPS tracks (compatible with many navigation apps)
        if format_type in ["gpx", "all"]:
            gpx_path = os.path.join(export_dir, f"{base_filename}.gpx")
            
            # Get GPS data
            conn = sqlite3.connect(self.db_path)
            df = pd.read_sql_query(
                "SELECT timestamp, latitude, longitude, quality, quality_score FROM road_quality",
                conn
            )
            conn.close()
            
            # Create GPX file
            gpx = gpxpy.gpx.GPX()
            
            # Create track
            gpx_track = gpxpy.gpx.GPXTrack()
            gpx_track.name = f"Road Quality Trip {self.trip_id}"
            gpx.tracks.append(gpx_track)
            
            # Create track segment
            gpx_segment = gpxpy.gpx.GPXTrackSegment()
            gpx_track.segments.append(gpx_segment)
            
            # Add track points
            for _, row in df.iterrows():
                if row['latitude'] and row['longitude']:
                    point = gpxpy.gpx.GPXTrackPoint(
                        latitude=row['latitude'],
                        longitude=row['longitude'],
                        time=datetime.fromisoformat(row['timestamp']) if row['timestamp'] else None
                    )
                    # Add quality data as extensions
                    point.extensions = f"""
                    <extensions>
                        <quality>{row['quality']}</quality>
                        <quality_score>{row['quality_score']}</quality_score>
                    </extensions>
                    """
                    gpx_segment.points.append(point)
            
            # Write GPX file
            with open(gpx_path, 'w') as f:
                f.write(gpx.to_xml())
                
            export_results["files"].append(gpx_path)
            logger.info(f"Exported GPX track to {gpx_path}")
        
        # Mark export as successful
        export_results["success"] = True
        logger.info(f"Successfully exported trip data in {format_type} format(s)")
        
        return export_results
        
    except Exception as e:
        logger.error(f"Error exporting trip data: {e}")
        export_results["error"] = str(e)
        return export_results
        
def update_quality_plot(self, frame):
    """Update function for road quality visualization"""
    with self.road_quality_lock:
        # Update quality score history plot
        if self.road_quality_history:
            quality_data = np.array(self.road_quality_history)
            x_data = np.arange(len(quality_data))
            self.quality_line.set_data(x_data, quality_data)
            
            # Adjust x-axis limits for scrolling effect
            self.ax_quality.set_xlim(max(0, len(quality_data) - 50), max(50, len(quality_data)))
            
            # Update quality score text
            current_quality = self.road_quality["quality"]
            current_score = self.road_quality["quality_score"]
            self.quality_text.set_text(f"Current Quality: {current_quality} ({current_score}/100)")
            
            # Set color based on quality
            if current_score >= 80:
                color = 'green'
            elif current_score >= 60:
                color = 'lightgreen'
            elif current_score >= 40:
                color = 'gold'
            elif current_score >= 20:
                color = 'orange'
            else:
                color = 'red'
            
            self.quality_text.set_color(color)
        
        # Update road profile visualization
        if self.road_profile_history:
            # Get the latest profile data
            latest_profile = self.road_profile_history[-1]
            
            # Create a 2D representation for the heatmap
            profile_data = np.zeros((10, len(latest_profile)))
            for i in range(10):  # Create 10 rows with the same data
                profile_data[i, :] = latest_profile
                
            # Update the road profile image
            self.road_profile.set_array(profile_data)
            
            # Update color limits for better contrast
            vmax = max(5, np.max(profile_data))
            vmin = min(-5, np.min(profile_data))
            self.road_profile.set_clim(vmin, vmax)
    
    # Return all artists that were modified
    return [self.quality_line, self.quality_text, self.road_profile]

def setup_road_quality_visualization(self):
    """Set up visualization specifically for road quality"""
    # Create a figure for road quality metrics
    self.fig_quality, (self.ax_quality, self.ax_profile) = plt.subplots(2, 1, figsize=(10, 8))
    self.fig_quality.canvas.manager.set_window_title('Road Quality Metrics - SensorFusion')
    
    # Make figure minimizable
    try:
        plt.rcParams['figure.figsize'] = [10, 8]
        
        if hasattr(self.fig_quality.canvas.manager, 'window'):
            backend = plt.get_backend().lower()
            if 'qt' in backend:
                self.fig_quality.canvas.manager.window.setWindowFlags(
                    self.fig_quality.canvas.manager.window.windowFlags() & ~0x00000020
                )
            elif 'tk' in backend:
                self.fig_quality.canvas.manager.window.wm_attributes("-topmost", 0)
    except Exception as e:
        logger.warning(f"Could not configure window manager for quality plot: {e}")
    
    # Top plot: Road quality score over time (0-100)
    self.quality_line, = self.ax_quality.plot([], [], 'g-', linewidth=2)
    self.ax_quality.set_ylim(0, 100)
    self.ax_quality.set_xlim(0, 50)  # Start with showing 50 points
    self.ax_quality.set_title("Road Quality Score")
    self.ax_quality.set_ylabel("Quality (0-100)")
    self.ax_quality.set_xlabel("Measurements")
    self.ax_quality.grid(True)
    
    # Add horizontal regions for quality categories
    self.ax_quality.axhspan(80, 100, alpha=0.2, color='green', label='Excellent')
    self.ax_quality.axhspan(60, 80, alpha=0.2, color='lightgreen', label='Good')
    self.ax_quality.axhspan(40, 60, alpha=0.2, color='gold', label='Fair')
    self.ax_quality.axhspan(20, 40, alpha=0.2, color='orange', label='Poor')
    self.ax_quality.axhspan(0, 20, alpha=0.2, color='red', label='Very Poor')
    self.ax_quality.legend(loc='upper right')
    
    # Add a text element to show current quality score
    self.quality_text = self.ax_quality.text(0.02, 0.92, "Quality: Unknown", 
                                            transform=self.ax_quality.transAxes,
                                            fontsize=12, fontweight='bold')
    
    # Bottom plot: Road profile from LiDAR
    # Create custom colormap for road profile (red=depression, green=bump)
    colors = [(0.8, 0, 0), (0.95, 0.95, 0.95), (0, 0.8, 0)]  # red, white, green
    n_bins = 100
    cmap_name = 'road_quality_cmap'
    custom_cmap = LinearSegmentedColormap.from_list(cmap_name, colors, N=n_bins)
    
    # Create placeholder data
    road_data = np.zeros((10, 100))
    
    # Create the heatmap for road profile
    self.road_profile = self.ax_profile.imshow(
        road_data,
        aspect='auto',
        cmap=custom_cmap,
        interpolation='nearest',
        origin='lower',
        extent=[0, 100, 0, 10]
    )
    
    # Add a colorbar
    cbar = self.fig_quality.colorbar(self.road_profile, ax=self.ax_profile)
    cbar.set_label('Surface Variation (mm)')
    
    self.ax_profile.set_title("Road Surface Profile")
    self.ax_profile.set_xlabel("Distance (samples)")
    self.ax_profile.set_ylabel("Width")
    self.ax_profile.set_yticks([])  # Hide y-axis ticks since they don't represent anything
    
    # Add user info text
    user_info = f"User: {self.config.USER_LOGIN} | Session: {self.config.SYSTEM_START_TIME}"
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    self.fig_quality.text(0.99, 0.01, f"{user_info} | Current time: {current_time}", 
                          horizontalalignment='right',
                          verticalalignment='bottom',
                          transform=self.fig_quality.transFigure,
                          fontsize=8, alpha=0.7)
    
    self.fig_quality.tight_layout()
    
    # Create animation
    self.quality_ani = animation.FuncAnimation(
        self.fig_quality,
        self.update_quality_plot,
        interval=self.config.UPDATE_INTERVAL * 2,  # Slightly slower update for this visualization
        blit=True,
        cache_frame_data=False
    )
    
    logger.info("Road quality visualization setup complete")

def calibrate_sensors(self):
    """Perform sensor calibration routine at startup"""
    logger.info("Starting sensor calibration...")
    
    # Step 1: Calibrate accelerometer to detect orientation
    accel_samples = []
    for _ in range(50):  # Collect 50 samples
        accel_z = self.get_accel_data()
        if accel_z is not None:
            accel_samples.append(accel_z)
        time.sleep(0.01)
    
    if accel_samples:
        # Calculate baseline gravitational acceleration
        self.accel_baseline = np.mean(accel_samples)
        logger.info(f"Accelerometer baseline: {self.accel_baseline:.3f}g")
        
        # Detect orientation based on baseline
        if 0.8 < self.accel_baseline < 1.2:
            self.orientation = "horizontal"
        elif -0.2 < self.accel_baseline < 0.2:
            self.orientation = "vertical"
        else:
            self.orientation = "tilted"
            
        logger.info(f"Detected orientation: {self.orientation}")
    else:
        logger.warning("Could not calibrate accelerometer")
        self.accel_baseline = 1.0
        self.orientation = "unknown"
    
    # Step 2: Calibrate LiDAR distance to ground (stationary)
    lidar_samples = []
    logger.info("Calibrating LiDAR ground distance. Keep the bike stationary...")
    
    for _ in range(10):  # Collect 10 scans
        try:
            scan_data = self.lidar_device.get_scan_as_vectors(filter_quality=True)
            filtered_data = self.filter_lidar_angles(scan_data)
            
            # Get the median distance for the points directly below
            distances = [point[1] for point in filtered_data 
                        if 355 <= (point[0] % 360) <= 360 or 0 <= (point[0] % 360) <= 5]
            
            if distances:
                lidar_samples.append(np.median(distances))
            time.sleep(0.1)
        except Exception as e:
            logger.error(f"Error during LiDAR calibration: {e}")
    
    if lidar_samples:
        # Set the ground distance baseline
        self.ground_distance_baseline = np.median(lidar_samples)
        logger.info(f"Ground distance baseline: {self.ground_distance_baseline:.2f}mm")
    else:
        logger.warning("Could not calibrate LiDAR ground distance")
        self.ground_distance_baseline = 300.0  # Default 30cm assumption
    
    # Store calibration data
    self.calibration_data = {
        "timestamp": datetime.now().isoformat(),
        "accel_baseline": float(self.accel_baseline),
        "ground_distance": float(self.ground_distance_baseline),
        "orientation": self.orientation
    }
    
    logger.info("Sensor calibration completed")
    return True

def detect_road_anomalies(self, lidar_data):
    """Detect specific road anomalies like potholes and bumps"""
    # Extract perpendicular distance measurements (ground points)
    ground_points = []
    for point in lidar_data:
        angle, distance = point
        angle_norm = angle % 360
        # Consider only points aimed close to perpendicular to ground
        if 355 <= angle_norm <= 359 or 0 <= angle_norm <= 5:
            ground_points.append(distance)
    
    if len(ground_points) < 5:
        return {"anomalies": [], "has_anomaly": False}
    
    # Calculate baseline (median distance)
    ground_points = np.array(ground_points)
    median_baseline = np.median(ground_points)
    
    # Define thresholds for anomalies (in mm)
    POTHOLE_THRESHOLD = 30    # 3cm deeper than surroundings
    BUMP_THRESHOLD = 20       # 2cm higher than surroundings
    CRACK_THRESHOLD = 15      # 1.5cm variation
    
    # Detect anomalies
    anomalies = []
    
    # Look for potholes (significantly farther distance)
    pothole_indices = np.where(ground_points - median_baseline > POTHOLE_THRESHOLD)[0]
    if len(pothole_indices) > 0:
        depth = np.max(ground_points[pothole_indices] - median_baseline)
        width = len(pothole_indices) * 2  # Approximate width in cm
        anomalies.append({
            "type": "pothole",
            "depth_mm": float(depth),
            "width_cm": width,
            "severity": "high" if depth > 50 else "medium"
        })
    
    # Look for bumps (significantly closer distance)
    bump_indices = np.where(median_baseline - ground_points > BUMP_THRESHOLD)[0]
    if len(bump_indices) > 0:
        height = np.max(median_baseline - ground_points[bump_indices])
        width = len(bump_indices) * 2  # Approximate width in cm
        anomalies.append({
            "type": "bump",
            "height_mm": float(height),
            "width_cm": width,
            "severity": "high" if height > 30 else "medium"
        })
    
    # Look for cracks (rapid variations)
    if len(ground_points) > 3:
        diffs = np.abs(np.diff(ground_points))
        crack_indices = np.where(diffs > CRACK_THRESHOLD)[0]
        if len(crack_indices) > 0:
            anomalies.append({
                "type": "crack",
                "width_mm": float(np.max(diffs[crack_indices])),
                "severity": "medium"
            })
    
    # Return all detected anomalies
    return {
        "anomalies": anomalies,
        "has_anomaly": len(anomalies) > 0,
        "baseline_mm": float(median_baseline)
    }

def process_road_data(self):
    """Process LiDAR and accelerometer data for road quality analysis"""
    # Get current LiDAR data with lock
    with self.lidar_data_lock:
        lidar_data = self.lidar_data.copy() if self.lidar_data else []
    
    # Get current accelerometer data with lock
    with self.accel_data_lock:
        accel_data = list(self.accel_data) if self.accel_data else []
    
    # Apply motion compensation
    compensated_data = self.apply_motion_compensation(lidar_data, accel_data)
    
    # Measure road quality
    road_quality = self.measure_road_quality(compensated_data)
    
    # Detect road anomalies
    anomalies = self.detect_road_anomalies(compensated_data)
    
    # Get current GPS position
    with self.gps_data_lock:
        gps_data = self.gps_data.copy()
    
    # Combine all data into a comprehensive road analysis
    analysis = {
        "timestamp": datetime.now().isoformat(),
        "gps": {
            "lat": gps_data.get("lat", 0),
            "lon": gps_data.get("lon", 0),
            "alt": gps_data.get("alt", 0)
        },
        "quality": road_quality,
        "anomalies": anomalies,
        "trip_id": self.trip_id
    }
    
    # Store analysis to memory for visualization
    with self.road_quality_lock:
        self.road_quality = road_quality
        self.road_anomalies = anomalies
    
    # Record data if recording is enabled
    if self.is_recording:
        self.record_data_point(analysis)
    
    # Check if we need to alert the user about hazards
    if anomalies.get("has_anomaly", False):
        self.alert_user_about_hazards(anomalies)
    
    return analysis

def apply_motion_compensation(self, lidar_data, accel_data):
    """Apply motion compensation to LiDAR readings based on accelerometer data"""
    # Skip if no accelerometer data is available
    if not accel_data or not hasattr(self, 'accel_baseline'):
        return lidar_data
    
    # Calculate vertical displacement from accelerometer
    current_accel = accel_data[-1] - self.accel_baseline  # Remove gravity component
    time_delta = 0.1  # Approximate time between samples (100ms)
    
    # Convert acceleration to m/s² (from g)
    accel_ms2 = current_accel * 9.81
    
    # Calculate displacement in mm
    displacement_mm = 0.5 * accel_ms2 * (time_delta ** 2) * 1000
    
    # Apply displacement compensation to each LiDAR reading
    compensated_data = []
    for point in lidar_data:
        angle, distance = point
        
        # Only compensate points that are measuring the ground
        angle_normalized = angle % 360
        if 0 <= angle_normalized <= 10 or 350 <= angle_normalized <= 359:
            # Adjust distance by calculated displacement
            adjusted_distance = distance - displacement_mm
            compensated_data.append((angle, adjusted_distance))
        else:
            # Keep other angles unchanged
            compensated_data.append(point)
    
    return compensated_data

def alert_user_about_hazards(self, anomalies):
    """Alert the user about detected road hazards"""
    if not anomalies or not anomalies.get("anomalies"):
        return
    
    # Log the alert
    anomaly_types = [a.get("type", "unknown") for a in anomalies.get("anomalies", [])]
    logger.warning(f"ROAD HAZARD DETECTED: {', '.join(anomaly_types)}")
    
    # In a real system, you could trigger visual or audio warnings here
    # For example:
    # - Flash an LED
    # - Play a warning sound
    # - Send a push notification
    
    # For our visualization, we'll update the road quality plot with an alert
    # (This is handled in the update_quality_plot method)
    pass

def road_quality_thread_func(self):
    """Thread function for continuous road quality analysis"""
    logger.info("Road quality analysis thread started")
    
    while not self.stop_event.is_set():
        try:
            # Process current data to calculate road quality
            analysis = self.process_road_data()
            
            # Save data periodically
            current_time = time.time()
            if hasattr(self, 'last_save_time') and (current_time - self.last_save_time > self.config.SAVE_INTERVAL):
                self.save_data()
                self.last_save_time = current_time
            
        except Exception as e:
            logger.error(f"Error in road quality thread: {e}")
        
        # Sleep to prevent high CPU usage
        time.sleep(0.5)
    
    logger.info("Road quality analysis thread stopped")

def run(self):
    """
    Main entry point to start all sensor threads, visualization, and processing loops.
    """
    try:
        logger.info("Starting SensorFusion run sequence...")

        # Initialize sensors
        self.initialize_lidar()
        self.initialize_gps()
        self.initialize_icm20948()
        self.calibrate_sensors()
        self.setup_recording()
        self.setup_battery_monitoring()
        self.setup_road_quality_visualization()
        self.setup_visualization()

        # Create and start threads for each sensor/process
        lidar_thread = Thread(target=self.lidar_thread_func, daemon=True)
        gps_thread = Thread(target=self.gps_thread_func, daemon=True)
        accel_thread = Thread(target=self.accel_thread_func, daemon=True)
        quality_thread = Thread(target=self.road_quality_thread_func, daemon=True)
        battery_thread = Thread(target=self.battery_monitoring_thread, daemon=True)

        lidar_thread.start()
        gps_thread.start()
        accel_thread.start()
        quality_thread.start()
        battery_thread.start()

        logger.info("All threads successfully started.")

        # Launch Matplotlib animation or any main event loop
        self.ani_lidar = FuncAnimation(
            self.lidar_fig, self.update_lidar_plot, fargs=(self.lidar_line,), interval=self.Config.UPDATE_INTERVAL
        )
        self.ani_accel = FuncAnimation(
            self.accel_fig, self.update_accel_plot, fargs=(self.accel_line,), interval=self.Config.UPDATE_INTERVAL
        )
        self.ani_quality = FuncAnimation(
            self.quality_fig, self.update_quality_plot, interval=self.Config.UPDATE_INTERVAL
        )
        
        plt.show()  # Blocking call until visualization window is closed

        # Once the plot window is closed, stop threads gracefully
        logger.info("Stopping SensorFusion threads...")
        self.stop_event.set()

        lidar_thread.join()
        gps_thread.join()
        accel_thread.join()
        quality_thread.join()
        battery_thread.join()

        logger.info("All threads terminated. Performing final data save...")
        self.save_data()
        logger.info("SensorFusion run sequence completed.")

    except KeyboardInterrupt:
        logger.warning("KeyboardInterrupt received; cleaning up...")
        self.stop_event.set()
        self.save_data()
    except Exception as e:
        logger.error(f"An error occurred in run: {e}", exc_info=True)
        self.stop_event.set()
        self.save_data()
