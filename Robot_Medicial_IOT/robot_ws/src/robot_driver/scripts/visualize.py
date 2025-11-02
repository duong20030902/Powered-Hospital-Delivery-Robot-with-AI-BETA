#!/usr/bin/env python3
# filepath: h:\Robot\company\slam_ws\my_ws\src\omni_base_driver\scripts\visualize.py
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import glob
from datetime import datetime

class STM32DataVisualizer:
    def __init__(self, csv_file_path=None):
        """Initialize the visualizer with CSV file path"""
        if csv_file_path is None:
            csv_file_path = self.find_newest_csv()
        
        self.csv_file_path = csv_file_path
        self.data = None
        self.load_data()
        self.calculate_distance()
    
    def find_newest_csv(self):
        """Find the newest STM32 CSV file in home directory"""
        home_dir = os.path.expanduser("~")
        
        # Search patterns for STM32 CSV files
        search_patterns = [
            os.path.join(home_dir, "stm32_data*.csv"),
            os.path.join(home_dir, "stm32*.csv"),
            os.path.join(home_dir, "*stm32*.csv"),
            os.path.join(home_dir, "*.csv")
        ]
        
        csv_files = []
        for pattern in search_patterns:
            csv_files.extend(glob.glob(pattern))
        
        if not csv_files:
            raise FileNotFoundError(f"No CSV files found in {home_dir}")
        
        # Sort by modification time (newest first)
        csv_files.sort(key=os.path.getmtime, reverse=True)
        
        newest_file = csv_files[0]
        print(f"Found {len(csv_files)} CSV files in home directory")
        print(f"Using newest file: {newest_file}")
        print(f"File modified: {datetime.fromtimestamp(os.path.getmtime(newest_file))}")
        
        return newest_file
    
    def load_data(self):
        """Load CSV data"""
        try:
            self.data = pd.read_csv(self.csv_file_path)
            
            # Convert timestamp to datetime
            if 'timestamp' in self.data.columns:
                self.data['datetime'] = pd.to_datetime(self.data['timestamp'], unit='s')
                # Calculate relative time in seconds from start
                self.data['time_relative'] = self.data['timestamp'] - self.data['timestamp'].iloc[0]
            
            print(f"Loaded {len(self.data)} data points from {self.csv_file_path}")
            print(f"Columns: {list(self.data.columns)}")
            
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            raise
    
    def calculate_distance(self):
        """Calculate distance traveled from wheel velocities (already in m/s)"""
        if 'v_left' not in self.data.columns or 'v_right' not in self.data.columns:
            print("Warning: Cannot calculate distance - missing wheel velocity data")
            return
        
        # Calculate time differences
        if 'timestamp' in self.data.columns and len(self.data) > 1:
            dt = np.diff(self.data['timestamp'])
            dt = np.insert(dt, 0, 0)  # Insert 0 for first element
            
            # Wheel velocities are already in m/s, so calculate robot velocity directly
            v_robot = (self.data['v_left'] + self.data['v_right']) / 2.0  # Average velocity in m/s
            
            # Calculate incremental distances
            distance_increments = v_robot * dt  # m/s * s = meters
            
            # Calculate cumulative distance
            self.data['distance_traveled'] = np.cumsum(np.abs(distance_increments))
            
            print(f"Distance calculation completed:")
            print(f"Total distance traveled: {self.data['distance_traveled'].iloc[-1]:.3f} meters")
        else:
            print("Warning: Cannot calculate distance - missing or insufficient timestamp data")
    
    def plot_distance(self):
        """Plot only distance traveled"""
        fig, ax = plt.subplots(1, 1, figsize=(12, 6))
        
        if 'distance_traveled' in self.data.columns:
            ax.plot(self.data['time_relative'], self.data['distance_traveled'], 'b-', linewidth=3, alpha=0.8)
            ax.fill_between(self.data['time_relative'], self.data['distance_traveled'], alpha=0.3, color='blue')
            
            ax.set_xlabel('Time (s)', fontsize=14)
            ax.set_ylabel('Distance (m)', fontsize=14)
            ax.set_title(f'Distance Traveled - {os.path.basename(self.csv_file_path)}', fontsize=16)
            ax.grid(True, alpha=0.3)
            
            # Add final distance annotation
            final_distance = self.data['distance_traveled'].iloc[-1]
            ax.annotate(f'Total Distance: {final_distance:.3f}m', 
                       xy=(self.data['time_relative'].iloc[-1], final_distance),
                       xytext=(0.6, 0.9), textcoords='axes fraction',
                       bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8),
                       arrowprops=dict(arrowstyle='->', color='blue', lw=2),
                       fontsize=14, fontweight='bold')
            
            # Add statistics text
            duration = self.data['time_relative'].iloc[-1]
            avg_speed = final_distance / duration if duration > 0 else 0
            
            stats_text = f"""
Distance Statistics:
• Total: {final_distance:.3f} m
• Duration: {duration:.1f} s
• Avg Speed: {avg_speed:.3f} m/s
• Data Points: {len(self.data)}
            """
            
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
                   fontsize=12, verticalalignment='top', fontfamily='monospace',
                   bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.9))
        else:
            ax.text(0.5, 0.5, 'No distance data available\nMissing wheel velocity data', 
                   ha='center', va='center', transform=ax.transAxes, fontsize=16)
            ax.set_title('Distance Analysis - No Data Available')
        
        plt.tight_layout()
        return fig

def main():
    parser = argparse.ArgumentParser(description='Calculate Distance from STM32 Wheel Velocity Data')
    parser.add_argument('csv_file', nargs='?', help='Path to CSV file (optional - will auto-find newest)')
    
    args = parser.parse_args()
    
    try:
        # If no file specified, auto-find newest
        if args.csv_file and os.path.exists(args.csv_file):
            visualizer = STM32DataVisualizer(args.csv_file)
        else:
            if args.csv_file:
                print(f"Warning: Specified file {args.csv_file} not found. Auto-finding newest...")
            visualizer = STM32DataVisualizer()  # Auto-find newest
        
        # Show distance plot
        visualizer.plot_distance()
        plt.show()
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()