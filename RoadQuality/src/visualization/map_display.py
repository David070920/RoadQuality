import folium
from folium.plugins import MarkerCluster
import logging

# Configure logging
logger = logging.getLogger("MapDisplay")

class MapDisplay:
    def __init__(self, initial_location=[0, 0], zoom_start=15):
        self.map = folium.Map(location=initial_location, zoom_start=zoom_start)
        self.marker_cluster = MarkerCluster().add_to(self.map)

    def add_marker(self, location, popup_text):
        try:
            folium.Marker(
                location=location,
                popup=folium.Popup(popup_text, max_width=300),
                icon=folium.Icon(color='blue')
            ).add_to(self.marker_cluster)
        except Exception as e:
            print(f"Error adding marker: {e}")

    def update_map(self, trip_coords, trip_quality):
        try:
            for i in range(len(trip_coords) - 1):
                color = self.get_color_based_on_quality(trip_quality[i])
                folium.PolyLine(
                    [trip_coords[i], trip_coords[i + 1]],
                    color=color,
                    weight=5,
                    opacity=0.8
                ).add_to(self.map)
        except Exception as e:
            print(f"Error updating map: {e}")

    def get_color_based_on_quality(self, quality_score):
        if quality_score >= 80:
            return 'green'
        elif quality_score >= 60:
            return 'lightgreen'
        elif quality_score >= 40:
            return 'yellow'
        elif quality_score >= 20:
            return 'orange'
        else:
            return 'red'

    def save_map(self, file_path):
        try:
            self.map.save(file_path)
        except Exception as e:
            print(f"Error saving map to {file_path}: {e}")

    def clear_markers(self):
        """Clear all markers from the map."""
        try:
            self.marker_cluster = MarkerCluster().add_to(self.map)
            logger.info("Cleared all markers from the map.")
        except Exception as e:
            print(f"Error clearing markers: {e}")

    def create_default_map(self):
        try:
            self.map = folium.Map(location=[0, 0], zoom_start=2)
            folium.Marker(
                [0, 0],
                popup="Waiting for GPS data...",
                icon=folium.Icon(color='red')
            ).add_to(self.map)
            self.save_map("default_map.html")
        except Exception as e:
            print(f"Error creating default map: {e}")