#!/usr/bin/env python3
"""
Real-time data monitor window for PyBullet simulation
Shows simulation statistics in a separate tkinter window
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import json
import os

class DataMonitor:
    """Real-time data monitoring window"""
    
    def __init__(self, title="Simulation Monitor"):
        self.title = title
        self.running = False
        self.window = None
        self.labels = {}
        self.data_file = "/tmp/pybullet_sim_data.json"
        self.update_interval = 0.5  # Update every 500ms
        
    def start(self):
        """Start the monitor window in a separate thread"""
        if self.running:
            return
            
        self.running = True
        self.monitor_thread = threading.Thread(target=self._run_monitor, daemon=True)
        self.monitor_thread.start()
        
    def stop(self):
        """Stop the monitor window"""
        self.running = False
        if self.window:
            try:
                self.window.quit()
                self.window.destroy()
            except:
                pass
                
    def _run_monitor(self):
        """Run the tkinter monitor window"""
        self.window = tk.Tk()
        self.window.title(self.title)
        self.window.geometry("400x300")
        self.window.configure(bg='black')
        
        # Create main frame
        main_frame = ttk.Frame(self.window, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure style for better visibility
        style = ttk.Style()
        style.configure('Monitor.TLabel', foreground='lime', background='black', font=('Courier', 10))
        
        # Create labels for different data fields
        self.labels = {}
        fields = [
            "Simulation Time",
            "Real Time", 
            "Target Speed",
            "Actual Speed",
            "Time Step",
            "Physics",
            "Robots",
            "Collisions",
            "Steps"
        ]
        
        for i, field in enumerate(fields):
            label = ttk.Label(main_frame, text=f"{field}: --", style='Monitor.TLabel')
            label.grid(row=i, column=0, sticky=tk.W, pady=2)
            self.labels[field] = label
            
        # Add status label
        self.status_label = ttk.Label(main_frame, text="Status: Waiting for data...", 
                                     style='Monitor.TLabel')
        self.status_label.grid(row=len(fields), column=0, sticky=tk.W, pady=10)
        
        # Start periodic update
        self.window.after(int(self.update_interval * 1000), self._update_display)
        
        # Start the tkinter main loop
        try:
            self.window.mainloop()
        except:
            pass
        finally:
            self.running = False
            
    def _update_display(self):
        """Update the display with latest data"""
        if not self.running:
            return
            
        try:
            # Read data from shared file
            if os.path.exists(self.data_file):
                with open(self.data_file, 'r') as f:
                    data = json.load(f)
                    
                # Update labels
                self.labels["Simulation Time"].config(text=f"Simulation Time: {data.get('sim_time', 0):.1f}s")
                self.labels["Real Time"].config(text=f"Real Time: {data.get('real_time', 0):.1f}s")
                self.labels["Target Speed"].config(text=f"Target Speed: {data.get('target_speed', 0):.1f}x")
                self.labels["Actual Speed"].config(text=f"Actual Speed: {data.get('actual_speed', 0):.1f}x")
                self.labels["Time Step"].config(text=f"Time Step: {data.get('time_step', 0):.4f}s ({data.get('frequency', 0):.0f}Hz)")
                self.labels["Physics"].config(text=f"Physics: {data.get('physics', 'Unknown')}")
                self.labels["Robots"].config(text=f"Robots: {data.get('robots', {})}")
                self.labels["Collisions"].config(text=f"Collisions: {data.get('collisions', 0)}")
                self.labels["Steps"].config(text=f"Steps: {data.get('steps', 0)}")
                
                self.status_label.config(text=f"Status: Connected (Updated: {time.strftime('%H:%M:%S')})")
            else:
                self.status_label.config(text="Status: No data file found")
                
        except Exception as e:
            self.status_label.config(text=f"Status: Error reading data - {str(e)}")
            
        # Schedule next update
        if self.running:
            self.window.after(int(self.update_interval * 1000), self._update_display)
            
    def write_data(self, sim_data):
        """Write simulation data to shared file (called from main simulation)"""
        try:
            with open(self.data_file, 'w') as f:
                json.dump(sim_data, f)
        except:
            pass  # Ignore errors in data writing

# Standalone monitor launcher
def main():
    """Run standalone data monitor"""
    monitor = DataMonitor("PyBullet Simulation Monitor")
    print("Starting data monitor window...")
    print("Press Ctrl+C to exit")
    
    try:
        monitor.start()
        # Keep main thread alive
        while monitor.running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping monitor...")
        monitor.stop()

if __name__ == "__main__":
    main()
    # This file has been moved to the core directory