import unittest
from src.analysis.road_quality import RoadQualityAnalyzer
from src.analysis.anomaly_detection import AnomalyDetector

class TestRoadQualityAnalyzer(unittest.TestCase):
    
    def setUp(self):
        self.config = type('Config', (object,), {
            "MIN_VALID_SAMPLES": 10,
            "LIDAR_MIN_DISTANCE": 50,
            "LIDAR_MAX_DISTANCE": 1000,
            "ROUGHNESS_THRESHOLDS": {
                "excellent": 1.0,
                "good": 3.0,
                "fair": 7.0,
                "poor": 15.0
            }
        })
        self.analyzer = RoadQualityAnalyzer(lidar_data=[], config=self.config)
    
    def test_measure_road_quality_insufficient_data(self):
        self.analyzer.lidar_data = [(0, 30), (10, 40)]  # Less than MIN_VALID_SAMPLES
        result = self.analyzer.measure_road_quality()
        self.assertEqual(result['quality'], "Unknown")

    def test_measure_road_quality_valid_data(self):
        self.analyzer.lidar_data = [(0, 100), (10, 110), (20, 105), (30, 95), (40, 100)]
        result = self.analyzer.measure_road_quality()
        self.assertIn('quality', result)
        self.assertIn('quality_score', result)

    def test_log_metrics(self):
        metrics = {
            "quality": "Good",
            "quality_score": 75,
            "roughness": 2.5,
            "variance": 1.2,
            "distance": 100
        }
        self.analyzer.log_metrics(metrics)  # Ensure no exceptions are raised

class TestAnomalyDetector(unittest.TestCase):
    
    def setUp(self):
        self.detector = AnomalyDetector(lidar_data=[])

    def test_detect_road_anomalies_no_data(self):
        anomalies = self.detector.detect_road_anomalies()
        self.assertEqual(len(anomalies), 0)

    def test_detect_road_anomalies_with_data(self):
        self.detector.lidar_data = [(0, 100), (10, 150), (20, 90), (30, 200)]
        anomalies = self.detector.detect_road_anomalies()
        self.assertGreater(len(anomalies), 0)

if __name__ == '__main__':
    unittest.main()