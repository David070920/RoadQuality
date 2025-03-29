import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import logging

# Configure logging
logger = logging.getLogger("RealTimePlots")

class RealTimePlots:
    def __init__(self, max_data_points=100):
        self.max_data_points = max_data_points
        self.lidar_data = deque(maxlen=max_data_points)
        self.accel_data = deque(maxlen=max_data_points)
        self.quality_data = deque(maxlen=max_data_points)

        # Create subplots
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8))
        self.lidar_line, = self.axs[0].plot([], [], label='LiDAR Data', color='blue')
        self.accel_line, = self.axs[1].plot([], [], label='Accelerometer Data', color='orange')
        self.quality_line, = self.axs[2].plot([], [], label='Road Quality Score', color='green')

        # Set titles and labels
        self.axs[0].set_title('Real-Time LiDAR Data')
        self.axs[0].set_xlabel('Angle (degrees)')
        self.axs[0].set_ylabel('Distance (mm)')
        self.axs[0].legend()
        
        self.axs[1].set_title('Real-Time Accelerometer Data')
        self.axs[1].set_xlabel('Sample Number')
        self.axs[1].set_ylabel('Acceleration (g)')
        self.axs[1].legend()
        
        self.axs[2].set_title('Real-Time Road Quality Score')
        self.axs[2].set_xlabel('Sample Number')
        self.axs[2].set_ylabel('Quality Score')
        self.axs[2].legend()

        plt.tight_layout()

    def update_lidar_plot(self, angles, distances):
        try:
            self.lidar_data.extend(zip(angles, distances))
            if len(self.lidar_data) > 0:
                angles, distances = zip(*self.lidar_data)
                self.lidar_line.set_data(angles, distances)
                self.axs[0].relim()
                self.axs[0].autoscale_view()
        except Exception as e:
            print(f"Error updating LiDAR plot: {e}")

    def update_accel_plot(self, accel_values):
        try:
            self.accel_data.extend(accel_values)
            if len(self.accel_data) > 0:
                self.accel_line.set_ydata(self.accel_data)
                self.accel_line.set_xdata(np.arange(len(self.accel_data)))
                self.axs[1].relim()
                self.axs[1].autoscale_view()
        except Exception as e:
            print(f"Error updating accelerometer plot: {e}")

    def update_quality_plot(self, quality_scores):
        try:
            self.quality_data.extend(quality_scores)
            if len(self.quality_data) > 0:
                self.quality_line.set_ydata(self.quality_data)
                self.quality_line.set_xdata(np.arange(len(self.quality_data)))
                self.axs[2].relim()
                self.axs[2].autoscale_view()
        except Exception as e:
            print(f"Error updating quality plot: {e}")

    def show(self):
        try:
            plt.show(block=False)
        except Exception as e:
            print(f"Error displaying plots: {e}")

    def reset_plots(self):
        """Reset all plots to their initial state."""
        try:
            self.lidar_data.clear()
            self.accel_data.clear()
            self.quality_data.clear()
            self.lidar_line.set_data([], [])
            self.accel_line.set_ydata([])
            self.quality_line.set_ydata([])
            for ax in self.axs:
                ax.relim()
                ax.autoscale_view()
            logger.info("All plots have been reset.")
        except Exception as e:
            print(f"Error resetting plots: {e}")

    def pause(self, interval):
        try:
            if interval <= 0:
                raise ValueError("Interval must be greater than 0.")
            plt.pause(interval)
        except Exception as e:
            print(f"Error pausing plots: {e}")