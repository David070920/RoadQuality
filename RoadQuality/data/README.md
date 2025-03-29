# Data Directory README

This directory contains the data collected from the road quality measurement project. The data includes various sensor readings from the LiDAR, GPS, and accelerometer, which are used to analyze and visualize road quality.

## Data Structure

The data is organized into timestamped directories for each trip, containing the following files:

- **road_quality.csv**: A CSV file that logs the road quality metrics, including timestamps, GPS coordinates, quality classifications, and roughness indices.
- **trip_data.db**: An SQLite database that stores structured data related to the trip, including road quality, accelerometer data, and trip metadata.
- **maps/**: A directory containing HTML files of the road quality maps generated during the trip.

## Usage

To utilize the collected data:

1. Access the `road_quality.csv` file for a quick overview of the road quality metrics.
2. Use the `trip_data.db` for detailed analysis and querying of the data.
3. Open the HTML files in the `maps/` directory to visualize the road quality on a map.

Ensure that you have the necessary permissions to read the files and that any required libraries for data analysis and visualization are installed.