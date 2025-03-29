import os
from pathlib import Path

class Config:
    MAX_DATA_POINTS = 100  # Maximum data points to store in memory
    UPDATE_INTERVAL = 10   # Visualization update interval in ms
    GPS_MAP_UPDATE_INTERVAL = 10   # GPS map update interval in seconds
    SAVE_INTERVAL = 30     # Data save interval in seconds

    USER_LOGIN = "David070920"
    SYSTEM_START_TIME = "2025-03-29 12:31:27"

    LIDAR_PORT = '/dev/ttyUSB0'
    LIDAR_DMAX = 4000      # Maximum LiDAR distance
    LIDAR_SCAN_MODE = 0    # Scan mode (0-2)

    LIDAR_MIN_ANGLE = -45  # Minimum display angle (converted from 315° to -45° for polar plot)
    LIDAR_MAX_ANGLE = 45   # Maximum display angle
    LIDAR_FILTER_ANGLES = [(0, 45), (315, 360)]  # Angles to keep (min, max)

    ROAD_QUALITY_WINDOW = 20       # Number of samples to analyze for road quality
    MIN_VALID_SAMPLES = 10         # Minimum samples needed for valid measurement
    ROUGHNESS_THRESHOLDS = {       # Thresholds for quality classification
        "excellent": 1.0,
        "good": 3.0,
        "fair": 7.0,
        "poor": 15.0
    }

    GPS_PORT = '/dev/ttyACM0'
    GPS_BAUD_RATE = 9600
    GPS_TIMEOUT = 0.5

    ICM20948_ADDRESS = 0x69
    ICM20948_WHO_AM_I = 0x00
    ICM20948_PWR_MGMT_1 = 0x06
    ICM20948_ACCEL_ZOUT_H = 0x31

    DATA_ROOT_DIR = os.path.join(str(Path.home()), "road_quality_data")

    MAP_HTML_PATH = os.path.join(str(Path.home()), "road_quality_map.html")
    MAP_ZOOM_START = 15

    @staticmethod
    def update_config(key, value):
        """Update a configuration value dynamically."""
        try:
            if hasattr(Config, key):
                setattr(Config, key, value)
                print(f"Updated configuration: {key} = {value}")
            else:
                raise AttributeError(f"Configuration key '{key}' does not exist.")
        except Exception as e:
            print(f"Error updating configuration: {e}")

    @staticmethod
    def validate():
        """Validate configuration values."""
        try:
            assert Config.MAX_DATA_POINTS > 0, "MAX_DATA_POINTS must be greater than 0"
            assert Config.UPDATE_INTERVAL > 0, "UPDATE_INTERVAL must be greater than 0"
            assert Config.SAVE_INTERVAL > 0, "SAVE_INTERVAL must be greater than 0"
            assert Config.LIDAR_DMAX > 0, "LIDAR_DMAX must be greater than 0"
            assert Config.GPS_BAUD_RATE > 0, "GPS_BAUD_RATE must be greater than 0"
            print("Configuration validated successfully.")
        except AssertionError as e:
            print(f"Configuration validation error: {e}")