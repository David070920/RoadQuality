import unittest
from src.sensors.lidar_sensor import LidarSensor
from src.sensors.gps_sensor import GPSSensor
from src.sensors.accelerometer import Accelerometer

class TestLidarSensor(unittest.TestCase):
    def setUp(self):
        self.lidar = LidarSensor(port='/dev/ttyUSB0')

    def test_initialization(self):
        self.assertTrue(self.lidar.initialize())

    def test_get_scan_data(self):
        self.lidar.initialize()
        scan_data = self.lidar.get_scan_data()
        self.assertIsInstance(scan_data, list)
        self.assertGreater(len(scan_data), 0)

    def test_reset(self):
        self.lidar.initialize()
        self.assertTrue(self.lidar.reset())

class TestGPSSensor(unittest.TestCase):
    def setUp(self):
        self.gps = GPSSensor(port='/dev/ttyACM0', baud_rate=9600)

    def test_initialization(self):
        self.assertTrue(self.gps.initialize())

    def test_get_location(self):
        self.gps.initialize()
        location = self.gps.read_data()
        self.assertIn('lat', location)
        self.assertIn('lon', location)

    def test_reset(self):
        self.gps.initialize()
        self.assertTrue(self.gps.reset())

class TestAccelerometer(unittest.TestCase):
    def setUp(self):
        self.accelerometer = Accelerometer(address=0x69, i2c_bus=None)

    def test_initialization(self):
        self.assertTrue(self.accelerometer.initialize())

    def test_get_acceleration(self):
        self.accelerometer.initialize()
        accel_data = self.accelerometer.get_acceleration()
        self.assertIsInstance(accel_data, tuple)
        self.assertEqual(len(accel_data), 3)

    def test_reset(self):
        self.accelerometer.initialize()
        self.assertTrue(self.accelerometer.reset())

if __name__ == '__main__':
    unittest.main()