import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import logging
from datetime import datetime

logger = logging.getLogger("SensorFusion")

def update_lidar_plot(num, line, lidar_data, lidar_data_lock, config):
    """Update function for LiDAR animation"""
    with lidar_data_lock:
        if not lidar_data:
            line.set_offsets([[], []])  # Clear previous points
            return line,
            
        # Process the data for visualization
        polar_data = []
        for point in lidar_data:
            angle_deg = point[0]
            distance = point[1]
            
            # Convert to radians for the polar plot
            angle_rad = np.radians(angle_deg)
            polar_data.append((angle_rad, distance))
        
        if not polar_data:
            line.set_offsets([[], []])  # Clear previous points
            return line,
            
        # Convert data to numpy arrays for faster processing
        angles = np.array([point[0] for point in polar_data])
        distances = np.array([point[1] for point in polar_data])
        
        # Update the plot
        offsets = np.column_stack((angles, distances))
        line.set_offsets(offsets)
        
        # Color by intensity/distance
        intensity = np.array([0 + (50 - 0) * (d / config.LIDAR_DMAX) for d in distances])
        line.set_array(intensity)
        
        # Clear previous points (this line ensures old data is not displayed)
        line.set_offsets([[], []])
        
    return line,

def update_accel_plot(frame, accel_line, accel_data, accel_data_lock, config):
    """Update function for accelerometer animation"""
    with accel_data_lock:
        if not accel_data:
            return accel_line,
            
        # Update the plot with current data
        data_array = np.array(accel_data)
        accel_line.set_ydata(data_array)
        
        # Adjust x-axis for proper scrolling effect
        accel_line.set_xdata(np.arange(len(data_array)))
        
    return accel_line,

def setup_visualization(lidar_data, lidar_data_lock, accel_data, accel_data_lock, config):
    """Set up matplotlib figures and animations"""
    # Set the matplotlib backend properties to allow window management
    # This works across different backends
    plt.rcParams['figure.raise_window'] = False
    
    # User info text for plot titles
    user_info = f"User: {config.USER_LOGIN} | Session: {config.SYSTEM_START_TIME}"
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    # LiDAR visualization
    fig_lidar, ax_lidar = plt.subplots(subplot_kw={'polar': True}, figsize=(8, 8))
    fig_lidar.canvas.manager.set_window_title('LiDAR Data - SensorFusion')
    
    # Make figure minimizable - using a cross-platform approach
    try:
        # Different backends might have different window management methods
        # Some might use 'window' attribute, others might not
        plt.rcParams['figure.figsize'] = [8, 8]
        plt.rcParams['figure.dpi'] = 100
        
        # Try to detect Qt backend and handle it properly
        if hasattr(fig_lidar.canvas.manager, 'window'):
            backend = plt.get_backend().lower()
            if 'qt' in backend:
                # For Qt backends
                fig_lidar.canvas.manager.window.setWindowFlags(
                    fig_lidar.canvas.manager.window.windowFlags() & ~0x00000020
                )
            elif 'tk' in backend:
                # For Tk backends 
                fig_lidar.canvas.manager.window.wm_attributes("-topmost", 0)
            # For other backends, we rely on the default behavior
        
        logger.info(f"Configured LiDAR visualization window using backend: {plt.get_backend()}")
    except Exception as e:
        logger.warning(f"Could not configure window manager for LiDAR plot: {e}")
    
    line = ax_lidar.scatter([0, 0], [0, 0], s=5, c=[0, 0], cmap=plt.cm.Greys_r, lw=0)
    
    ax_lidar.set_rmax(1000)  # Set maximum distance to display
    
    # Set the angle limits to show the full 360-degree field of view
    ax_lidar.set_thetamin(0)
    ax_lidar.set_thetamax(360)
    
    ax_lidar.grid(True)
    ax_lidar.set_title(f"LiDAR Data (360° FOV)\n{user_info}")
    
    lidar_ani = animation.FuncAnimation(
        fig_lidar, 
        update_lidar_plot,
        fargs=(line, lidar_data, lidar_data_lock, config), 
        interval=config.UPDATE_INTERVAL, 
        blit=True,
        cache_frame_data=False
    )
    
    # Accelerometer visualization
    fig_accel, ax_accel = plt.subplots(figsize=(10, 4))
    fig_accel.canvas.manager.set_window_title('Accelerometer Data - SensorFusion')
    
    # Make figure minimizable - similar approach for accelerometer plot
    try:
        plt.rcParams['figure.figsize'] = [10, 4]
        
        if hasattr(fig_accel.canvas.manager, 'window'):
            backend = plt.get_backend().lower()
            if 'qt' in backend:
                fig_accel.canvas.manager.window.setWindowFlags(
                    fig_accel.canvas.manager.window.windowFlags() & ~0x00000020
                )
            elif 'tk' in backend:
                fig_accel.canvas.manager.window.wm_attributes ("-topmost", 0)
        
        logger.info(f"Configured accelerometer visualization window")
    except Exception as e:
        logger.warning(f"Could not configure window manager for accelerometer plot: {e}")
    
    # Initialize with empty data
    accel_line, = ax_accel.plot(
        np.arange(config.MAX_DATA_POINTS),
        np.zeros(config.MAX_DATA_POINTS),
        'b-', 
        label='Acceleration (Z)'
    )
    
    ax_accel.set_xlim(0, config.MAX_DATA_POINTS - 1)
    ax_accel.set_ylim(-2, 2)
    ax_accel.set_title(f"Accelerometer Data\n{user_info}")
    ax_accel.set_xlabel("Sample")
    ax_accel.set_ylabel("Acceleration (g)")
    ax_accel.grid(True)
    ax_accel.legend(loc='upper right')
    
    # Add user info text in the lower right corner
    fig_accel.text(0.99, 0.01, f"{user_info} | Current time: {current_time}", 
                   horizontalalignment='right',
                   verticalalignment='bottom',
                   transform=fig_accel.transFigure,
                   fontsize=8, alpha=0.7)
    
    accel_ani = animation.FuncAnimation(
        fig_accel, 
        update_accel_plot, 
        fargs=(accel_line, accel_data, accel_data_lock, config),
        interval=config.UPDATE_INTERVAL, 
        blit=True,
        cache_frame_data=False
    )
    
    return fig_lidar, fig_accel, lidar_ani, accel_ani
