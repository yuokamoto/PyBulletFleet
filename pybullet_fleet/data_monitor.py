#!/usr/bin/env python3
"""
Real-time data monitor window for PyBullet simulation
Shows simulation statistics in a separate tkinter window
"""

import json
import os
import threading
import time
import tkinter as tk
from tkinter import ttk
from typing import Dict, Optional

from pybullet_fleet._defaults import SIMULATION as _SIM_D


class DataMonitor:
    """Real-time data monitoring window"""

    def __init__(
        self,
        title: str = "Simulation Monitor",
        enable_gui: bool = True,
        width: int = _SIM_D["monitor_width"],
        height: int = _SIM_D["monitor_height"],
        x: int = _SIM_D["monitor_x"],
        y: int = _SIM_D["monitor_y"],
    ) -> None:
        self.title: str = title
        self.enable_gui: bool = enable_gui
        self.running: bool = False
        self.window: Optional[tk.Tk] = None
        self.labels: Dict[str, ttk.Label] = {}
        self.data_file: str = "/tmp/pybullet_sim_data.json"
        self.update_interval: float = 0.5  # Update every 500ms
        self.status_label: Optional[ttk.Label] = None
        self.monitor_thread: Optional[threading.Thread] = None
        # If x/y are -1, omit position so the window manager places the window
        # on the primary display. Explicit x/y use absolute virtual-screen coords.
        if x >= 0 and y >= 0:
            self.geometry_string: str = f"{width}x{height}+{x}+{y}"
        else:
            self.geometry_string: str = f"{width}x{height}"

    def start(self) -> None:
        """Start the monitor window in a separate thread"""
        if self.running:
            return

        self.running = True

        # Only start GUI thread if GUI is enabled
        if self.enable_gui:
            self.monitor_thread = threading.Thread(target=self._run_monitor, daemon=True)
            self.monitor_thread.start()

    def stop(self) -> None:
        """Stop the monitor window"""
        self.running = False
        if self.window:
            try:
                self.window.quit()
                self.window.destroy()
            except Exception:
                pass

    def _run_monitor(self) -> None:
        """Run the tkinter monitor window"""
        self.window = tk.Tk()
        self.window.title(self.title)
        self.window.geometry(self.geometry_string)
        self.window.configure(bg="black")

        # Create main frame  (padding: top right bottom left)
        main_frame = ttk.Frame(self.window, padding="4 5 4 2")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure style for better visibility
        style = ttk.Style()
        style.configure("Monitor.TLabel", foreground="lime", background="black", font=("Courier", 9))

        # Create labels for different data fields
        self.labels = {}
        fields = [
            "Sim Time",
            "Real Time",
            "Target RTF",
            "Actual RTF",
            "Time Step",
            "Physics",
            "Agents",
            "Objects",
            "Collisions",
            "Tot Collis.",
            "Steps",
        ]

        for i, field in enumerate(fields):
            label = ttk.Label(main_frame, text=f"{field}: --", style="Monitor.TLabel")
            label.grid(row=i, column=0, sticky=tk.W, pady=2)
            self.labels[field] = label

        # Add status label
        self.status_label = ttk.Label(main_frame, text="Status: Waiting for data...", style="Monitor.TLabel")
        self.status_label.grid(row=len(fields), column=0, sticky=tk.W, pady=10)

        # Start periodic update
        self.window.after(int(self.update_interval * 1000), self._update_display)

        # Start the tkinter main loop
        try:
            self.window.mainloop()
        except Exception:
            pass
        finally:
            self.running = False

    def _update_display(self) -> None:
        """Update the display with latest data"""
        if not self.running:
            return

        try:
            # Read data from shared file
            if os.path.exists(self.data_file):
                with open(self.data_file, "r") as f:
                    data = json.load(f)

                # Update labels
                self.labels["Sim Time"].config(text=f"Sim Time : {data.get('sim_time', 0):.1f}s")
                self.labels["Real Time"].config(text=f"Real Time: {data.get('real_time', 0):.1f}s")
                self.labels["Target RTF"].config(text=f"Tgt RTF  : {data.get('target_rtf', 0):.1f}x")
                self.labels["Actual RTF"].config(text=f"Act RTF  : {data.get('actual_rtf', 0):.1f}x")
                self.labels["Time Step"].config(text=f"dt={data.get('time_step', 0):.4f}s ({data.get('frequency', 0):.0f}Hz)")
                self.labels["Physics"].config(text=f"Physics  : {data.get('physics', '?')}")
                self.labels["Agents"].config(text=f"Agents   : {data.get('agents', 0)}")
                self.labels["Objects"].config(text=f"Objects  : {data.get('objects', 0)}")
                self.labels["Collisions"].config(text=f"Collis.  : {data.get('active_collisions', 0)}")
                self.labels["Tot Collis."].config(text=f"Tot Col. : {data.get('collisions', 0)}")
                self.labels["Steps"].config(text=f"Steps    : {data.get('steps', 0)}")

                self.status_label.config(text=f"Status: Connected (Updated: {time.strftime('%H:%M:%S')})")
            else:
                self.status_label.config(text="Status: No data file found")

        except Exception as e:
            self.status_label.config(text=f"Status: Error reading data - {str(e)}")

        # Schedule next update
        if self.running:
            self.window.after(int(self.update_interval * 1000), self._update_display)

    def write_data(self, sim_data):
        """Write simulation data to shared file (called from main simulation).

        Uses atomic write (write to temp file + os.replace) to prevent the
        reader from seeing a truncated / partially-written JSON file.
        """
        tmp_path = self.data_file + ".tmp"
        try:
            with open(tmp_path, "w") as f:
                json.dump(sim_data, f)
            os.replace(tmp_path, self.data_file)  # atomic on POSIX
        except Exception:
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
