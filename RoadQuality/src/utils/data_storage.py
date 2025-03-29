import os
import json
import sqlite3
import pickle
import logging
import time
import zipfile

# Configure logging
logger = logging.getLogger("DataStorage")

class DataStorage:
    def __init__(self, data_root_dir):
        self.data_root_dir = data_root_dir
        os.makedirs(self.data_root_dir, exist_ok=True)

    def save_to_json(self, data, filename):
        """Save data to a JSON file."""
        file_path = os.path.join(self.data_root_dir, filename)
        try:
            with open(file_path, 'w') as json_file:
                json.dump(data, json_file)
            logger.info(f"Data saved to JSON file: {file_path}")
        except Exception as e:
            logger.error(f"Failed to save data to JSON: {e}")
            raise

    def load_from_json(self, filename):
        """Load data from a JSON file."""
        file_path = os.path.join(self.data_root_dir, filename)
        try:
            with open(file_path, 'r') as json_file:
                data = json.load(json_file)
            logger.info(f"Data loaded from JSON file: {file_path}")
            return data
        except FileNotFoundError:
            logger.warning(f"JSON file not found: {file_path}")
            return None
        except Exception as e:
            logger.error(f"Failed to load data from JSON: {e}")
            raise

    def save_to_sqlite(self, db_name, table_name, data):
        """Save data to an SQLite database."""
        db_path = os.path.join(self.data_root_dir, db_name)
        try:
            conn = sqlite3.connect(db_path)
            c = conn.cursor()
            c.execute(f"CREATE TABLE IF NOT EXISTS {table_name} (id INTEGER PRIMARY KEY, data BLOB)")
            c.execute(f"INSERT INTO {table_name} (data) VALUES (?)", (pickle.dumps(data),))
            conn.commit()
            conn.close()
            logger.info(f"Data saved to SQLite database: {db_path}, table: {table_name}")
        except Exception as e:
            logger.error(f"Failed to save data to SQLite: {e}")

    def load_from_sqlite(self, db_name, table_name):
        """Load data from an SQLite database."""
        db_path = os.path.join(self.data_root_dir, db_name)
        try:
            conn = sqlite3.connect(db_path)
            c = conn.cursor()
            c.execute(f"SELECT data FROM {table_name}")
            rows = c.fetchall()
            conn.close()
            data = [pickle.loads(row[0]) for row in rows]
            logger.info(f"Data loaded from SQLite database: {db_path}, table: {table_name}")
            return data
        except Exception as e:
            logger.error(f"Failed to load data from SQLite: {e}")
            return None

    def archive_old_files(self, archive_name="archive.zip", days=30):
        """Archive files older than a specified number of days into a zip file."""
        archive_path = os.path.join(self.data_root_dir, archive_name)
        now = time.time()
        try:
            with zipfile.ZipFile(archive_path, 'w') as archive:
                for filename in os.listdir(self.data_root_dir):
                    file_path = os.path.join(self.data_root_dir, filename)
                    if os.path.isfile(file_path) and now - os.path.getmtime(file_path) > days * 86400:
                        archive.write(file_path, os.path.basename(file_path))
                        os.remove(file_path)
                        logger.info(f"Archived and deleted file: {file_path}")
            logger.info(f"Archived old files into {archive_path}")
        except Exception as e:
            logger.error(f"Failed to archive old files: {e}")

    def delete_old_files(self, days=30):
        """Delete files older than a specified number of days."""
        now = time.time()
        deleted_files = 0
        for filename in os.listdir(self.data_root_dir):
            file_path = os.path.join(self.data_root_dir, filename)
            if os.path.isfile(file_path) and now - os.path.getmtime(file_path) > days * 86400:
                try:
                    os.remove(file_path)
                    deleted_files += 1
                    logger.info(f"Deleted old file: {file_path}")
                except Exception as e:
                    logger.error(f"Failed to delete file {file_path}: {e}")
        logger.info(f"Deleted {deleted_files} old files.")