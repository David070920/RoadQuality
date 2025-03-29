# Road Quality Measurement Project

This project is designed to measure and analyze road quality using a Raspberry Pi equipped with various sensors, including a LiDAR sensor, GPS, and an accelerometer. The collected data is processed to evaluate road conditions and visualize the results in real-time.

## New Features
- **Reset Methods**: Added reset functionality for all sensors to handle errors gracefully.
- **Dynamic Configuration**: Update configuration values at runtime using the `Config.update_config` method.
- **Improved Logging**: Enhanced logging for better debugging and monitoring.

## Project Structure

```
RoadQuality
├── src
│   ├── display.py               # Main application for sensor fusion and visualization
│   ├── sensors                   # Contains sensor classes for LiDAR, GPS, and accelerometer
│   │   ├── __init__.py
│   │   ├── lidar_sensor.py       # LiDAR sensor handling
│   │   ├── gps_sensor.py         # GPS data acquisition
│   │   └── accelerometer.py      # Accelerometer data retrieval
│   ├── analysis                  # Contains analysis classes for road quality and anomaly detection
│   │   ├── __init__.py
│   │   ├── road_quality.py       # Road quality assessment
│   │   └── anomaly_detection.py   # Detection of road anomalies
│   ├── visualization             # Visualization tools for mapping and plotting
│   │   ├── __init__.py
│   │   ├── map_display.py        # Folium map visualization
│   │   └── realtime_plots.py     # Real-time plotting with matplotlib
│   └── utils                    # Utility functions for data handling and configuration
│       ├── __init__.py
│       ├── data_storage.py       # Functions for saving/loading data
│       └── config.py            # Configuration settings
├── data                          # Directory for storing collected data
│   └── README.md                 # Information about the data directory
├── tests                         # Unit tests for the project
│   ├── __init__.py
│   ├── test_sensors.py           # Tests for sensor classes
│   └── test_analysis.py          # Tests for analysis classes
├── requirements.txt              # Python dependencies
├── setup.py                      # Packaging and installation instructions
└── README.md                     # Project overview and usage guidelines
```

## Installation

1. Clone the repository:
   ```
   git clone <repository-url>
   cd RoadQuality
   ```

2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

3. Set up the Raspberry Pi with the necessary sensors connected.

## Usage

1. Run the main application:
   ```
   python src/display.py
   ```

2. The application will start collecting data from the sensors and visualizing road quality in real-time.

## Data Collection

The collected data will be stored in the `data` directory. This includes road quality metrics, GPS coordinates, and raw sensor data for further analysis.

## Testing

To run the unit tests, use:
```
pytest tests/
```

Ensure all dependencies are installed before running the tests.

## Contributing

Contributions are welcome! Please submit a pull request or open an issue for any suggestions or improvements.

## License

This project is licensed under the MIT License. See the LICENSE file for details.