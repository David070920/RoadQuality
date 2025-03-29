import time
import threading
import pynmea2
import logging
from utils import get_accel_data  # Import the get_accel_data function

logger = logging.getLogger("SensorFusion")

def filter_lidar_angles(scan_data, config):
    """Filter LiDAR data to only include points within specified angle ranges"""
    filtered_data = []
    
    for point in scan_data:
        angle = point[0]
        for angle_range in config.LIDAR_FILTER_ANGLES:
            if angle_range[0] <= angle <= angle_range[1]:
                filtered_data.append(point)
                break
                
    return filtered_data

def lidar_thread_func(lidar_device, lidar_data_lock, lidar_data, stop_event, config):
    """Thread function for LiDAR data acquisition"""
    logger.info("LiDAR thread started")
    while not stop_event.is_set():
        try:
            # Fetch the LiDAR scan data
            scan_data = lidar_device.get_scan_as_vectors(filter_quality=True)
            
            # Filter data based on angles
            filtered_data = filter_lidar_angles(scan_data, config)
            
            # Update shared data with lock
            with lidar_data_lock:
                lidar_data.extend(filtered_data)
                
        except Exception as e:
            logger.error(f"Error in LiDAR thread: {e}")
            
        # Sleep to prevent high CPU usage
        time.sleep(0.05)
    
    logger.info("LiDAR thread stopped")

def gps_thread_func(gps_serial_port, gps_data_lock, gps_data, stop_event, config, update_gps_map):
    """Thread function for GPS data acquisition"""
    logger.info("GPS thread started")
    while not stop_event.is_set():
        try:
            # Fetch the GPS data
            raw_data = gps_serial_port.readline().decode().strip()
            
            if raw_data.find('GGA') > 0:
                gps_message = pynmea2.parse(raw_data)
                
                # Update shared data with lock
                with gps_data_lock:
                    gps_data.update({
                        "timestamp": gps_message.timestamp,
                        "lat": round(gps_message.latitude, 6),
                        "lon": round(gps_message.longitude, 6),
                        "alt": gps_message.altitude,
                        "sats": gps_message.num_sats
                    })
                
                # Check if it's time to update the map
                current_time = time.time()
                if current_time - gps_data["last_map_update"] >= config.GPS_MAP_UPDATE_INTERVAL:
                    update_gps_map(gps_data, config)
                    gps_data["last_map_update"] = current_time
                
                logger.info(f"GPS: {gps_data}")
                
        except Exception as e:
            logger.debug(f"Error in GPS thread: {e}")
            
        # Sleep to prevent high CPU usage
        time.sleep(0.2)
        
    logger.info("GPS thread stopped")

def accel_thread_func(i2c_bus, accel_data_lock, accel_data, stop_event, config):
    """Thread function for accelerometer data acquisition"""
    logger.info("Accelerometer thread started")
    while not stop_event.is_set():
        try:
            # Get accelerometer data
            accel_z = get_accel_data(i2c_bus, config)
            
            if accel_z is not None:
                # Update shared data with lock
                with accel_data_lock:
                    accel_data.append(accel_z)
                
                logger.debug(f"Accelerometer: Z={accel_z:.2f}g")
                
        except Exception as e:
            logger.error(f"Error in accelerometer thread: {e}")
            
        # Sleep to prevent high CPU usage
        time.sleep(0.1)
        
    logger.info("Accelerometer thread stopped")
