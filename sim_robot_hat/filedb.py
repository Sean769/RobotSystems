import os

class fileDB:
    """A simulated or file-based database."""

    def __init__(self, db: str, mode: str = None, owner: str = None):  
        self.db = db
        self.simulated_data = {}  # Use this for simulation

    def get(self, name, default_value=None):
        """
        Get value with data's name.
        """
        return self.simulated_data.get(name, default_value)

    def set(self, name, value):
        """
        Set a value in the simulated database.
        """
        self.simulated_data[name] = value
