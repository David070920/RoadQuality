#!/usr/bin/env python3
"""
Main entry point for the Road Quality measurement application.
"""
import logging
import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

# Configure logging
logging.basicConfig(level=logging.INFO, 
                   format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")

# Import the main display class
from RoadQuality.src.display import Display

if __name__ == "__main__":
    # Create and run the display
    display = Display()
    display.run()
