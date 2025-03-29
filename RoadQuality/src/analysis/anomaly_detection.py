import numpy as np

class AnomalyDetector:
    def __init__(self, lidar_data):
        self.lidar_data = lidar_data

    def detect_road_anomalies(self):
        """Detect specific road anomalies like potholes and bumps."""
        ground_points = self.extract_ground_points()
        
        if len(ground_points) < 10:
            return []

        median_baseline = np.median(ground_points)
        deviations = abs(ground_points - median_baseline)

        anomalies = self.identify_anomalies(deviations, ground_points, median_baseline)
        return anomalies

    def extract_ground_points(self):
        """Extract ground points from LiDAR data."""
        ground_points = []
        for point in self.lidar_data:
            angle, distance = point
            if -45 <= angle <= 45 and distance > 50:  # Filter based on angle and distance
                ground_points.append(distance)
        return ground_points

    def identify_anomalies(self, deviations, ground_points, median_baseline):
        """Identify anomalies based on deviation thresholds."""
        anomalies = []
        POTHOLE_THRESHOLD = 30  # 3cm deeper than surroundings
        BUMP_THRESHOLD = 20      # 2cm higher than surroundings

        for i, deviation in enumerate(deviations):
            if deviation > POTHOLE_THRESHOLD and ground_points[i] < median_baseline:
                anomalies.append((i, 'Pothole', ground_points[i]))
            elif deviation > BUMP_THRESHOLD and ground_points[i] > median_baseline:
                anomalies.append((i, 'Bump', ground_points[i]))

        return anomalies

    def _cluster_anomalies(self, anomalies, max_gap=2):
        """Cluster detected anomalies based on proximity."""
        clustered_anomalies = []
        current_cluster = []

        for anomaly in anomalies:
            if not current_cluster:
                current_cluster.append(anomaly)
            else:
                if anomaly[0] - current_cluster[-1][0] <= max_gap:
                    current_cluster.append(anomaly)
                else:
                    clustered_anomalies.append(current_cluster)
                    current_cluster = [anomaly]

        if current_cluster:
            clustered_anomalies.append(current_cluster)

        return clustered_anomalies

    def alert_user_about_hazards(self, anomalies):
        """Alert the user about detected road hazards."""
        for index, anomaly_type, distance in anomalies:
            print(f"Alert: Detected a {anomaly_type} at index {index} with distance {distance:.2f} cm.")