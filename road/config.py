import os
from pathlib import Path

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