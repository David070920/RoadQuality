import time
import folium
import os
import logging

logger = logging.getLogger("SensorFusion")

def read_byte(i2c_bus, addr, reg):
    """Read a byte from the I2C device"""
    retries = 3
    for _ in range(retries):
        try:
            return i2c_bus.read_byte_data(addr, reg)
        except Exception as e:
            logger.debug(f"Error reading byte from address 0x{addr:02x}, register 0x{reg:02x}: {e}")
            time.sleep(0.01)
    logger.warning(f"Failed to read from address 0x{addr:02x}, register 0x{reg:02x} after {retries} retries")
    return None

def read_word(i2c_bus, addr, reg):
    """Read a word from the I2C device"""
    high = read_byte(i2c_bus, addr, reg)
    low = read_byte(i2c_bus, addr, reg + 1)
    if high is not None and low is not None:
        val = (high << 8) + low
        return val
    else:
        return None

def read_word_2c(i2c_bus, addr, reg):
    """Read a 2's complement word from the I2C device"""
    val = read_word(i2c_bus, addr, reg)
    if val is not None:
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
    else:
        return None

def get_accel_data(i2c_bus, config):
    """Get accelerometer data from ICM20948"""
    accel_z = read_word_2c(i2c_bus, config.ICM20948_ADDRESS, config.ICM20948_ACCEL_ZOUT_H)
    if accel_z is not None:
        return accel_z / 16384.0  # Convert to g
    return None

def update_gps_map(gps_data, config):
    """Update the GPS position on a Folium map and save as HTML"""
    try:
        with gps_data["lock"]:
            lat = gps_data["lat"]
            lon = gps_data["lon"]
            alt = gps_data["alt"]
            sats = gps_data["sats"]
            timestamp = gps_data["timestamp"]
        
        # Skip if we don't have valid coordinates yet
        if lat == 0 and lon == 0:
            logger.warning("No valid GPS coordinates yet, skipping map update")
            return
                
        # Create a map centered at the GPS coordinates
        m = folium.Map(location=[lat, lon], zoom_start=config.MAP_ZOOM_START)
        
        # Add a marker for the current position
        popup_text = f"""
        <b>GPS Data</b><br>
        Latitude: {lat:.6f}°<br>
        Longitude: {lon:.6f}°<br>
        Altitude: {alt} m<br>
        Satellites: {sats}<br>
        Time: {timestamp}<br>
        <hr>
        User: {config.USER_LOGIN}<br>
        Session: {config.SYSTEM_START_TIME}
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
        m.save(config.MAP_HTML_PATH)
        logger.info(f"GPS map updated at {config.MAP_HTML_PATH}")
        
        # Verify the file exists
        if os.path.exists(config.MAP_HTML_PATH):
            logger.info(f"GPS map file successfully created")
        else:
            logger.error(f"Failed to create GPS map file")
            
    except Exception as e:
        logger.error(f"Error updating GPS map: {e}")
