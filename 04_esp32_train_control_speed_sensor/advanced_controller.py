import asyncio
from bleak import BleakClient, BleakScanner
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
from datetime import datetime
import json
import time

SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
CHAR_UUID = "87654321-4321-4321-4321-cba987654321"

class EnhancedTrainController:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Enhanced LEGO Train Controller v3.0 - With Rotation Sensor")
        self.root.geometry("1100x800")  # Increased height from 750 to 800
        self.root.configure(bg='#f0f0f0')
        
        # BLE Connection variables
        self.client = None
        self.esp32_address = None
        self.connected = False
        self.connection_lost = False
        self.last_ping_time = 0
        self.ping_interval = 5  # seconds
        
        # Train status variables
        self.hub_connected = False
        self.motor_speed = 0
        self.is_moving = False
        self.battery_level = -1
        self.hub_name = ""
        self.rssi = 0
        
        # Sensor status variables
        self.sensor_distance = 0.0
        self.sensor_speed = 0.0
        self.sensor_rpm = 0.0
        self.sensor_enabled = True
        self.last_sensor_update_time = 0
        
        # Distance movement tracking
        self.distance_mode = "SENSOR"
        self.distance_target = 0.0
        self.distance_traveled = 0.0
        self.distance_progress = 0.0
        
        # Timed movement tracking
        self.timed_duration = 0.0
        self.timed_elapsed = 0.0
        self.timed_progress = 0.0
        
        # Statistics
        self.total_distance = 0.0
        self.total_time = 0
        self.max_speed = 0
        self.total_movements = 0
        self.emergency_stop_active = False
        
        # GUI Variables
        self.speed_var = tk.IntVar(value=0)
        self.time_var = tk.IntVar(value=5)
        self.distance_var = tk.DoubleVar(value=50.0)
        self.movement_speed_var = tk.IntVar(value=50)
        
        self.setup_gui()
        
        # Start the asyncio event loop in a separate thread
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.run_async_loop, daemon=True)
        self.loop_thread.start()
        
    def run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
        
    def setup_gui(self):
        # Configure style
        style = ttk.Style()
        style.theme_use('clam')
        
        # Main container with notebook for tabs
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create notebook for tabbed interface
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Create tabs
        self.create_connection_tab()
        self.create_control_tab()
        self.create_advanced_tab()
        self.create_sensor_tab()  # New sensor tab
        self.create_led_tab()
        self.create_status_tab()
        self.create_log_tab()
        
        # Status bar at bottom
        self.status_bar = ttk.Label(self.root, text="Ready - Not Connected", 
                                   relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Emergency stop button (always visible at bottom)
        self.emergency_frame = ttk.Frame(self.root)
        self.emergency_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)
        
        self.emergency_btn = tk.Button(self.emergency_frame, text="🛑 EMERGENCY STOP", 
                                     command=self.emergency_stop,
                                     bg='red', fg='white', font=('Arial', 12, 'bold'),
                                     height=2)
        self.emergency_btn.pack(side=tk.LEFT, padx=5)
        
        self.clear_emergency_btn = tk.Button(self.emergency_frame, text="✅ Clear Emergency", 
                                           command=self.clear_emergency,
                                           bg='green', fg='white', font=('Arial', 10),
                                           state=tk.DISABLED)
        self.clear_emergency_btn.pack(side=tk.LEFT, padx=5)
        
    def create_connection_tab(self):
        # Connection Tab
        conn_tab = ttk.Frame(self.notebook)
        self.notebook.add(conn_tab, text="🔗 Connection")
        
        # Connection controls
        conn_frame = ttk.LabelFrame(conn_tab, text="BLE Connection")
        conn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Instructions
        instruction_frame = ttk.Frame(conn_frame)
        instruction_frame.pack(fill=tk.X, padx=10, pady=5)
        
        instructions = ttk.Label(instruction_frame, text="1. Click 'Scan for ESP32' → 2. Select device from list → 3. Click 'Connect'", 
                               font=('Arial', 9), foreground='blue')
        instructions.pack(anchor=tk.W)
        
        btn_frame = ttk.Frame(conn_frame)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.scan_btn = ttk.Button(btn_frame, text="🔍 Scan for ESP32", command=self.scan_for_esp32)
        self.scan_btn.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(btn_frame, text="🔗 Connect", command=self.connect_to_esp32, state=tk.DISABLED)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = ttk.Button(btn_frame, text="🔌 Disconnect", command=self.disconnect_from_esp32, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        # Device list
        device_frame = ttk.LabelFrame(conn_tab, text="Available Devices")
        device_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.device_tree = ttk.Treeview(device_frame, columns=('address', 'rssi'), show='tree headings')
        self.device_tree.heading('#0', text='Device Name')
        self.device_tree.heading('address', text='Address')
        self.device_tree.heading('rssi', text='RSSI')
        self.device_tree.bind('<<TreeviewSelect>>', self.on_device_selected)
        self.device_tree.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Connection status
        status_frame = ttk.LabelFrame(conn_tab, text="Connection Status")
        status_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.conn_status_label = ttk.Label(status_frame, text="ESP32: Disconnected")
        self.conn_status_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.hub_status_label = ttk.Label(status_frame, text="LEGO Hub: Disconnected")
        self.hub_status_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.sensor_status_label = ttk.Label(status_frame, text="Rotation Sensor: Unknown")
        self.sensor_status_label.pack(anchor=tk.W, padx=10, pady=5)
        
    def create_control_tab(self):
        # Control Tab
        control_tab = ttk.Frame(self.notebook)
        self.notebook.add(control_tab, text="🚂 Basic Control")
        
        # Speed control
        speed_frame = ttk.LabelFrame(control_tab, text="Motor Speed Control")
        speed_frame.pack(fill=tk.X, padx=10, pady=10)
        
        speed_control_frame = ttk.Frame(speed_frame)
        speed_control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(speed_control_frame, text="Speed (-100 to 100):").pack(side=tk.LEFT)
        
        self.speed_scale = ttk.Scale(speed_control_frame, from_=-100, to=100, 
                                   orient=tk.HORIZONTAL, variable=self.speed_var,
                                   command=self.on_speed_change)
        self.speed_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        self.speed_label = ttk.Label(speed_control_frame, text="0")
        self.speed_label.pack(side=tk.RIGHT)
        
        # Quick control buttons including emergency stop
        quick_frame = ttk.Frame(speed_frame)
        quick_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Large emergency stop button first
        emergency_stop_btn = tk.Button(quick_frame, text="🛑 EMERGENCY STOP", 
                                     command=self.emergency_stop,
                                     bg='red', fg='white', font=('Arial', 11, 'bold'),
                                     height=2, width=15)
        emergency_stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Regular control buttons
        ttk.Button(quick_frame, text="⏹️ Stop", command=self.stop_motor).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="⬅️ Reverse", command=lambda: self.set_speed(-50)).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="▶️ Forward", command=lambda: self.set_speed(50)).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="⚡ Full Forward", command=lambda: self.set_speed(100)).pack(side=tk.LEFT, padx=5)
        
        # Speed adjustment buttons
        adjust_frame = ttk.Frame(speed_frame)
        adjust_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(adjust_frame, text="-- -20", command=lambda: self.adjust_speed(-20)).pack(side=tk.LEFT, padx=2)
        ttk.Button(adjust_frame, text="- -10", command=lambda: self.adjust_speed(-10)).pack(side=tk.LEFT, padx=2)
        ttk.Button(adjust_frame, text="+ +10", command=lambda: self.adjust_speed(10)).pack(side=tk.LEFT, padx=2)
        ttk.Button(adjust_frame, text="++ +20", command=lambda: self.adjust_speed(20)).pack(side=tk.LEFT, padx=2)
        
        # Current status with sensor data
        current_frame = ttk.LabelFrame(control_tab, text="Current Status")
        current_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Emergency status frame
        emergency_status_frame = ttk.Frame(current_frame)
        emergency_status_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.emergency_status_label = ttk.Label(emergency_status_frame, text="Emergency Stop: Clear ✅", 
                                               font=('Arial', 10, 'bold'), foreground='green')
        self.emergency_status_label.pack(side=tk.LEFT)
        
        self.quick_clear_btn = tk.Button(emergency_status_frame, text="Clear Emergency", 
                                       command=self.clear_emergency, bg='orange', fg='white',
                                       state=tk.DISABLED)
        self.quick_clear_btn.pack(side=tk.RIGHT, padx=5)
        
        status_grid = ttk.Frame(current_frame)
        status_grid.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(status_grid, text="Motor Speed:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.current_speed_label = ttk.Label(status_grid, text="0", font=('Arial', 10, 'bold'))
        self.current_speed_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(status_grid, text="Movement:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        self.movement_label = ttk.Label(status_grid, text="Stopped", font=('Arial', 10, 'bold'))
        self.movement_label.grid(row=0, column=3, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(status_grid, text="Battery:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.battery_label = ttk.Label(status_grid, text="Unknown", font=('Arial', 10, 'bold'))
        self.battery_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(status_grid, text="Hub Name:").grid(row=1, column=2, sticky=tk.W, padx=5, pady=2)
        self.hub_name_label = ttk.Label(status_grid, text="Not Connected", font=('Arial', 10, 'bold'))
        self.hub_name_label.grid(row=1, column=3, sticky=tk.W, padx=5, pady=2)
        
        # Real-time sensor data
        ttk.Label(status_grid, text="Actual Speed:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        self.sensor_speed_label = ttk.Label(status_grid, text="0.0 cm/s", font=('Arial', 10, 'bold'), foreground='blue')
        self.sensor_speed_label.grid(row=2, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(status_grid, text="Distance:").grid(row=2, column=2, sticky=tk.W, padx=5, pady=2)
        self.sensor_distance_label = ttk.Label(status_grid, text="0.0 cm", font=('Arial', 10, 'bold'), foreground='blue')
        self.sensor_distance_label.grid(row=2, column=3, sticky=tk.W, padx=5, pady=2)
        
        # RPM display
        ttk.Label(status_grid, text="Wheel RPM:").grid(row=3, column=0, sticky=tk.W, padx=5, pady=2)
        self.sensor_rpm_label = ttk.Label(status_grid, text="0.0 RPM", font=('Arial', 10, 'bold'), foreground='orange')
        self.sensor_rpm_label.grid(row=3, column=1, sticky=tk.W, padx=5, pady=2)
        
    def create_advanced_tab(self):
        # Advanced Control Tab
        advanced_tab = ttk.Frame(self.notebook)
        self.notebook.add(advanced_tab, text="⚙️ Advanced")
        
        # Timed movement
        time_frame = ttk.LabelFrame(advanced_tab, text="Timed Movement (With Sensor Feedback)")
        time_frame.pack(fill=tk.X, padx=10, pady=10)
        
        time_controls = ttk.Frame(time_frame)
        time_controls.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(time_controls, text="Time (seconds):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        time_spin = ttk.Spinbox(time_controls, from_=1, to=300, textvariable=self.time_var, width=10)
        time_spin.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(time_controls, text="Speed:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=5)
        time_speed_spin = ttk.Spinbox(time_controls, from_=-100, to=100, textvariable=self.movement_speed_var, width=10)
        time_speed_spin.grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Button(time_controls, text="▶️ Start Timed Movement", command=self.start_timed_movement).grid(row=0, column=4, padx=10, pady=5)
        
        # Timed movement progress display
        timed_progress_frame = ttk.Frame(time_frame)
        timed_progress_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.timed_progress_label = ttk.Label(timed_progress_frame, text="Progress: Ready", font=('Arial', 10))
        self.timed_progress_label.pack(anchor=tk.W)
        
        self.timed_progress_bar = ttk.Progressbar(timed_progress_frame, length=400, mode='determinate')
        self.timed_progress_bar.pack(fill=tk.X, pady=5)
        
        # Timed movement info
        timed_info = ttk.Label(time_frame, text="""⏱️ Timed Movement: Runs for exact time duration while monitoring actual distance traveled
💡 Shows real distance covered during the time period for analysis and calibration""", 
                              font=('Arial', 9), foreground='blue', justify=tk.LEFT)
        timed_info.pack(anchor=tk.W, padx=10, pady=5)
        
        # Distance movement with sensor feedback toggle
        distance_frame = ttk.LabelFrame(advanced_tab, text="Distance-Based Movement (Sensor Feedback)")
        distance_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Distance mode toggle
        mode_frame = ttk.Frame(distance_frame)
        mode_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(mode_frame, text="Control Mode:", font=('Arial', 10, 'bold')).pack(side=tk.LEFT)
        self.distance_mode_var = tk.StringVar(value="SENSOR")
        self.distance_mode_label = ttk.Label(mode_frame, text="🎯 Sensor Feedback", font=('Arial', 10, 'bold'), foreground='green')
        self.distance_mode_label.pack(side=tk.LEFT, padx=10)
        
        ttk.Button(mode_frame, text="Toggle Mode", command=self.toggle_distance_mode).pack(side=tk.LEFT, padx=10)
        
        distance_controls = ttk.Frame(distance_frame)
        distance_controls.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(distance_controls, text="Distance (cm):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        distance_spin = ttk.Spinbox(distance_controls, from_=1, to=1000, textvariable=self.distance_var, width=10, increment=10)
        distance_spin.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(distance_controls, text="Speed:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=5)
        distance_speed_spin = ttk.Spinbox(distance_controls, from_=-100, to=100, textvariable=self.movement_speed_var, width=10)
        distance_speed_spin.grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Button(distance_controls, text="📏 Start Distance Movement", command=self.start_distance_movement).grid(row=0, column=4, padx=10, pady=5)
        
        # Distance progress display
        progress_frame = ttk.Frame(distance_frame)
        progress_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.distance_progress_label = ttk.Label(progress_frame, text="Progress: Ready", font=('Arial', 9))
        self.distance_progress_label.pack(anchor=tk.W)
        
        self.distance_progress_bar = ttk.Progressbar(progress_frame, length=500, mode='determinate')  # Increased width
        self.distance_progress_bar.pack(fill=tk.X, pady=5)
        
        # Distance mode info
        distance_info = ttk.Label(distance_frame, text="""🎯 Sensor Mode: Uses rotation sensor for precise distance control (recommended)
⏱️ Time Mode: Uses speed×time estimation (fallback when sensor unavailable)""", 
                                 font=('Arial', 9), foreground='blue', justify=tk.LEFT)
        distance_info.pack(anchor=tk.W, padx=10, pady=5)
        
        # Statistics with sensor comparison
        stats_frame = ttk.LabelFrame(advanced_tab, text="Movement Statistics - Estimated vs Actual")
        stats_frame.pack(fill=tk.X, padx=10, pady=10)
        
        stats_grid = ttk.Frame(stats_frame)
        stats_grid.pack(fill=tk.X, padx=10, pady=10)
        
        # Headers
        ttk.Label(stats_grid, text="Metric", font=('Arial', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        ttk.Label(stats_grid, text="Estimated", font=('Arial', 10, 'bold')).grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        ttk.Label(stats_grid, text="Actual (Sensor)", font=('Arial', 10, 'bold'), foreground='blue').grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        ttk.Label(stats_grid, text="Difference", font=('Arial', 10, 'bold')).grid(row=0, column=3, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(stats_grid, text="Total Distance:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.total_distance_label = ttk.Label(stats_grid, text="0.0 cm", font=('Arial', 10, 'bold'))
        self.total_distance_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)
        self.sensor_total_distance_label = ttk.Label(stats_grid, text="0.0 cm", font=('Arial', 10, 'bold'), foreground='blue')
        self.sensor_total_distance_label.grid(row=1, column=2, sticky=tk.W, padx=5, pady=2)
        self.distance_diff_label = ttk.Label(stats_grid, text="0.0 cm", font=('Arial', 10, 'bold'))
        self.distance_diff_label.grid(row=1, column=3, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(stats_grid, text="Total Time:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        self.total_time_label = ttk.Label(stats_grid, text="0 s", font=('Arial', 10, 'bold'))
        self.total_time_label.grid(row=2, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(stats_grid, text="Max Speed Used:").grid(row=3, column=0, sticky=tk.W, padx=5, pady=2)
        self.max_speed_label = ttk.Label(stats_grid, text="0", font=('Arial', 10, 'bold'))
        self.max_speed_label.grid(row=3, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(stats_grid, text="Total Movements:").grid(row=4, column=0, sticky=tk.W, padx=5, pady=2)
        self.total_movements_label = ttk.Label(stats_grid, text="0", font=('Arial', 10, 'bold'))
        self.total_movements_label.grid(row=4, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Button(stats_frame, text="🔄 Reset All Statistics", command=self.reset_statistics).pack(pady=5)
        
    def create_sensor_tab(self):
        # New Sensor Tab
        sensor_tab = ttk.Frame(self.notebook)
        self.notebook.add(sensor_tab, text="🎯 Rotation Sensor")
        
        # Sensor controls
        sensor_controls = ttk.LabelFrame(sensor_tab, text="Sensor Control")
        sensor_controls.pack(fill=tk.X, padx=10, pady=10)
        
        control_buttons = ttk.Frame(sensor_controls)
        control_buttons.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(control_buttons, text="📊 Get Sensor Data", command=self.get_sensor_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_buttons, text="🔄 Reset Sensor", command=self.reset_sensor).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_buttons, text="✅ Enable Sensor", command=self.enable_sensor).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_buttons, text="❌ Disable Sensor", command=self.disable_sensor).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_buttons, text="🔍 Debug Distance", command=self.debug_distance).pack(side=tk.LEFT, padx=5)
        
        # Real-time sensor display with live indicator
        sensor_display = ttk.LabelFrame(sensor_tab, text="Real-time Sensor Data (Live Updates)")
        sensor_display.pack(fill=tk.X, padx=10, pady=10)
        
        # Live indicator
        live_frame = ttk.Frame(sensor_display)
        live_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(live_frame, text="Status:", font=('Arial', 10, 'bold')).pack(side=tk.LEFT)
        self.live_indicator = ttk.Label(live_frame, text="🔴 Waiting...", font=('Arial', 10, 'bold'))
        self.live_indicator.pack(side=tk.LEFT, padx=5)
        
        self.last_update_label = ttk.Label(live_frame, text="Last update: Never", font=('Arial', 9))
        self.last_update_label.pack(side=tk.RIGHT)
        
        sensor_grid = ttk.Frame(sensor_display)
        sensor_grid.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(sensor_grid, text="Distance Traveled:", font=('Arial', 12, 'bold')).grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.display_distance_label = ttk.Label(sensor_grid, text="0.0 cm", font=('Arial', 14, 'bold'), foreground='blue')
        self.display_distance_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=5)
        
        ttk.Label(sensor_grid, text="Current Speed:", font=('Arial', 12, 'bold')).grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.display_speed_label = ttk.Label(sensor_grid, text="0.0 cm/s", font=('Arial', 14, 'bold'), foreground='green')
        self.display_speed_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=5)
        
        ttk.Label(sensor_grid, text="Wheel RPM:", font=('Arial', 12, 'bold')).grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        self.display_rpm_label = ttk.Label(sensor_grid, text="0.0 RPM", font=('Arial', 14, 'bold'), foreground='orange')
        self.display_rpm_label.grid(row=2, column=1, sticky=tk.W, padx=5, pady=5)
        
        # Sensor info with performance details
        sensor_info = ttk.LabelFrame(sensor_tab, text="Sensor Information & Performance")
        sensor_info.pack(fill=tk.X, padx=10, pady=10)
        
        info_text = ttk.Label(sensor_info, text="""🎯 Rotation Sensor Details:
• Pin: Digital Pin 2 (interrupt enabled)
• Wheel Radius: 8.25mm (0.825cm) 
• Sensor Type: 2 holes per wheel rotation
• Detection: 4 sensor changes per full rotation

⚡ Performance Optimization:
• ESP32 Update Rate: 50ms (20 Hz) 
• Data Transmission: 200ms (5 Hz) via BLE
• GUI Update Rate: 100ms (10 Hz) 
• Auto Status Refresh: 5 seconds

💡 Real-time sensor data provides actual movement feedback independent of motor commands,
enabling precise feedback and closed-loop control possibilities.""", 
                           font=('Arial', 9), foreground='blue', justify=tk.LEFT, wraplength=600)
        info_text.pack(anchor=tk.W, padx=10, pady=5)
        
    def create_led_tab(self):
        # LED Control Tab
        led_tab = ttk.Frame(self.notebook)
        self.notebook.add(led_tab, text="💡 LED Control")
        
        # ESP32 LED Control
        esp32_frame = ttk.LabelFrame(led_tab, text="ESP32 RGB LED Control")
        esp32_frame.pack(fill=tk.X, padx=10, pady=10)
        
        esp32_buttons = ttk.Frame(esp32_frame)
        esp32_buttons.pack(fill=tk.X, padx=10, pady=10)
        
        colors = [("Red", "ESP32_LED_RED"), ("Green", "ESP32_LED_GREEN"), ("Blue", "ESP32_LED_BLUE"),
                 ("Yellow", "ESP32_LED_YELLOW"), ("Purple", "ESP32_LED_PURPLE"), ("Cyan", "ESP32_LED_CYAN"),
                 ("White", "ESP32_LED_WHITE"), ("Off", "ESP32_LED_OFF"), ("Blink", "ESP32_LED_BLINK"), 
                 ("Auto Status", "ESP32_LED_AUTO")]
        
        for i, (text, cmd) in enumerate(colors):
            ttk.Button(esp32_buttons, text=text, command=lambda c=cmd: self.send_command(c)).grid(row=i//5, column=i%5, padx=5, pady=5)
        
        # LED info
        led_info = ttk.Label(esp32_frame, text="""💡 LED Control Logic:
• Colors (Red/Green/etc): Manual override for 10 seconds, then auto
• OFF: Turn off LED and return to automatic status immediately  
• Auto Status: Return to automatic status immediately""", 
                           font=('Arial', 9), foreground='blue', justify=tk.LEFT)
        led_info.pack(anchor=tk.W, padx=10, pady=5)
        
        # Hub LED Control
        hub_frame = ttk.LabelFrame(led_tab, text="LEGO Hub LED Control")
        hub_frame.pack(fill=tk.X, padx=10, pady=10)
        
        hub_buttons = ttk.Frame(hub_frame)
        hub_buttons.pack(fill=tk.X, padx=10, pady=10)
        
        hub_colors = [("Red", "HUB_LED_RED"), ("Green", "HUB_LED_GREEN"), ("Blue", "HUB_LED_BLUE"),
                     ("Yellow", "HUB_LED_YELLOW"), ("Purple", "HUB_LED_PURPLE"), ("White", "HUB_LED_WHITE"), ("Off", "HUB_LED_OFF")]
        
        for i, (text, cmd) in enumerate(hub_colors):
            ttk.Button(hub_buttons, text=text, command=lambda c=cmd: self.send_command(c)).grid(row=i//4, column=i%4, padx=5, pady=5)
        
    def create_status_tab(self):
        # Status Tab
        status_tab = ttk.Frame(self.notebook)
        self.notebook.add(status_tab, text="📊 Status")
        
        # Detailed status
        detail_frame = ttk.LabelFrame(status_tab, text="Detailed System Status")
        detail_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Auto-refresh controls
        refresh_frame = ttk.Frame(detail_frame)
        refresh_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(refresh_frame, text="🔄 Refresh Status", command=self.refresh_status).pack(side=tk.LEFT, padx=5)
        ttk.Button(refresh_frame, text="🔋 Get Battery", command=self.get_battery).pack(side=tk.LEFT, padx=5)
        ttk.Button(refresh_frame, text="🧪 Test Comm", command=self.test_communication).pack(side=tk.LEFT, padx=5)
        
        self.auto_refresh_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(refresh_frame, text="Auto-refresh (5s)", variable=self.auto_refresh_var).pack(side=tk.LEFT, padx=20)
        
        # Status display
        self.status_text = scrolledtext.ScrolledText(detail_frame, height=15, font=('Courier', 10))
        self.status_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Hub management
        hub_mgmt_frame = ttk.LabelFrame(status_tab, text="Hub Management")
        hub_mgmt_frame.pack(fill=tk.X, padx=10, pady=10)
        
        hub_buttons = ttk.Frame(hub_mgmt_frame)
        hub_buttons.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(hub_buttons, text="🔌 Disconnect Hub", command=self.disconnect_hub).pack(side=tk.LEFT, padx=5)
        ttk.Button(hub_buttons, text="🔗 Reconnect Hub", command=self.reconnect_hub).pack(side=tk.LEFT, padx=5)
        ttk.Button(hub_buttons, text="🔄 Force Hub Check", command=self.force_hub_check).pack(side=tk.LEFT, padx=5)
        
    def create_log_tab(self):
        # Log Tab
        log_tab = ttk.Frame(self.notebook)
        self.notebook.add(log_tab, text="📝 Log")
        
        # Log controls
        log_controls = ttk.Frame(log_tab)
        log_controls.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(log_controls, text="🗑️ Clear Log", command=self.clear_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(log_controls, text="💾 Save Log", command=self.save_log).pack(side=tk.LEFT, padx=5)
        
        # Log display
        self.log_text = scrolledtext.ScrolledText(log_tab, height=20, font=('Courier', 9))
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Start timers after GUI is fully initialized
        self.root.after(100, self.start_timers)
        
    def start_timers(self):
        """Start all timers after GUI initialization is complete"""
        self.auto_refresh_timer()
        self.fast_gui_update_timer()
        self.connection_monitor_timer()
        
    def auto_refresh_timer(self):
        if self.auto_refresh_var.get() and self.connected:
            self.log_message("Auto-refresh: Requesting status...", "DEBUG")
            self.refresh_status()
        self.root.after(5000, self.auto_refresh_timer)  # Every 5 seconds instead of 10
        
    def fast_gui_update_timer(self):
        """Fast update timer for real-time sensor display"""
        if self.connected:
            # Update GUI elements that need to be responsive
            self.update_sensor_display_only()
            
            # Check for sensor data timeout (no updates for 2 seconds)
            if hasattr(self, 'live_indicator') and self.last_sensor_update_time > 0:
                time_since_update = time.time() - self.last_sensor_update_time
                if time_since_update > 2.0:  # 2 second timeout
                    self.live_indicator.config(text="🟡 Timeout", foreground='orange')
                elif time_since_update > 5.0:  # 5 second timeout
                    self.live_indicator.config(text="🔴 No Data", foreground='red')
        else:
            # Disconnected
            if hasattr(self, 'live_indicator'):
                self.live_indicator.config(text="🔴 Disconnected", foreground='red')
                
        self.root.after(100, self.fast_gui_update_timer)  # Every 100ms for smooth updates
        
    def connection_monitor_timer(self):
        """Monitor connection health and detect disconnections"""
        current_time = time.time()
        
        if self.connected:
            # Check if we haven't heard from ESP32 in a while
            if current_time - self.last_ping_time > 15:  # 15 seconds timeout
                if not self.connection_lost:
                    self.log_message("Connection timeout detected - ESP32 may have disconnected", "WARNING")
                    self.connection_lost = True
                    # Try to detect actual disconnection
                    threading.Thread(target=self.check_connection_status, daemon=True).start()
            
            # Send periodic ping to keep connection alive
            elif current_time - self.last_ping_time > self.ping_interval:
                self.send_command("PING")
        
        self.root.after(3000, self.connection_monitor_timer)  # Every 3 seconds
    
    def check_connection_status(self):
        """Check if BLE connection is actually still alive"""
        try:
            if self.client and not self.client.is_connected:
                self.log_message("BLE connection lost - ESP32 disconnected", "ERROR")
                self.root.after(0, self.on_disconnected)
        except Exception as e:
            self.log_message(f"Connection check failed: {e}", "ERROR")
            self.root.after(0, self.on_disconnected)
        
    def log_message(self, message, level="INFO"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}\n"
        
        if hasattr(self, 'log_text'):
            self.log_text.insert(tk.END, log_entry)
            self.log_text.see(tk.END)
            
        print(log_entry.strip())
        
    def notification_handler(self, sender, data):
        try:
            message = data.decode('utf-8')
            self.log_message(f"Received: {message}", "RECV")
            self.parse_esp32_response(message)
        except UnicodeDecodeError:
            hex_data = ' '.join(f'{b:02x}' for b in data)
            self.log_message(f"Binary data received (possible corruption): {hex_data}", "WARNING")
            if self.connected:
                self.root.after(1000, lambda: self.send_command("GET_STATUS"))
    
    def parse_esp32_response(self, message):
        try:
            self.last_ping_time = time.time()
            self.connection_lost = False
            
            if message.startswith("STATUS:"):
                self.log_message(f"Parsing status: {message}", "DEBUG")
                parts = message[7:].split(',')
                status_dict = {}
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        status_dict[key] = value
                
                self.root.after(0, lambda: self.update_status_from_dict(status_dict))
                
            elif message.startswith("TIMED_PROGRESS:"):
                # Parse timed movement progress
                progress_data = message[15:]  # Remove "TIMED_PROGRESS:" prefix
                parts = progress_data.split(',')
                progress_dict = {}
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        progress_dict[key] = value
                
                self.log_message(f"Timed progress: {progress_dict}", "INFO")
                self.root.after(0, lambda: self.update_timed_progress(progress_dict))
                
            elif "TIMED_COMPLETED:" in message:
                # Parse timed completion data
                completion_data = message.split('TIMED_COMPLETED:')[1]
                self.log_message(f"Timed movement completed: {completion_data}", "INFO")
                self.root.after(0, lambda: self.handle_timed_completion(completion_data))
                
            elif message.startswith("DEBUG:"):
                # Parse debug information
                debug_data = message[6:]  # Remove "DEBUG:" prefix
                self.log_message(f"Debug info: {debug_data}", "DEBUG")
                
            elif message.startswith("DISTANCE_PROGRESS:"):
                # Parse distance movement progress
                progress_data = message[18:]  # Remove "DISTANCE_PROGRESS:" prefix
                parts = progress_data.split(',')
                progress_dict = {}
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        progress_dict[key] = value
                
                self.log_message(f"Distance progress: {progress_dict}", "INFO")
                self.root.after(0, lambda: self.update_distance_progress(progress_dict))
                
            elif message.startswith("DISTANCE_MODE:"):
                mode = message.split(':')[1]
                self.distance_mode = mode
                self.log_message(f"Distance mode: {mode}", "INFO")
                self.root.after(0, self.update_distance_mode_display)
                
            elif "DISTANCE_COMPLETED:" in message:
                # Parse distance completion data
                completion_data = message.split('DISTANCE_COMPLETED:')[1]
                self.log_message(f"Distance movement completed: {completion_data}", "INFO")
                
            elif message.startswith("SENSOR_LIVE:"):
                # Parse live sensor data (sent every 200ms)
                sensor_data = message[12:]  # Remove "SENSOR_LIVE:" prefix
                parts = sensor_data.split(',')
                sensor_dict = {}
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        sensor_dict[key] = value
                
                # Update sensor data without logging (too frequent)
                self.root.after(0, lambda: self.update_sensor_from_dict(sensor_dict))
                
            elif message.startswith("SENSOR_DATA:"):
                # Parse sensor data response
                sensor_data = message[12:]  # Remove "SENSOR_DATA:" prefix
                parts = sensor_data.split(',')
                sensor_dict = {}
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        sensor_dict[key] = value
                
                self.log_message(f"Sensor data: {sensor_dict}", "INFO")
                self.root.after(0, lambda: self.update_sensor_from_dict(sensor_dict))
                
            elif message.startswith("BATTERY_UPDATE:"):
                battery = int(message.split(':')[1])
                self.battery_level = battery
                self.log_message(f"Battery updated: {battery}%", "INFO")
                self.root.after(0, self.update_gui_status)
                
            elif message.startswith("RSSI_UPDATE:"):
                rssi = int(message.split(':')[1])
                self.rssi = rssi
                self.log_message(f"RSSI updated: {rssi} dB", "INFO")
                
            elif "EMERGENCY_STOP" in message:
                if "EMERGENCY_STOP_EXECUTED" in message:
                    self.emergency_stop_active = True
                elif "EMERGENCY_CLEARED" in message:
                    self.emergency_stop_active = False
                self.root.after(0, self.update_emergency_status)
                
            elif message == "PONG":
                pass
                
            elif message.startswith("ERROR:EMERGENCY_STOP_ACTIVE"):
                self.emergency_stop_active = True
                self.root.after(0, self.update_emergency_status)
                self.log_message("Movement blocked: Emergency stop is active!", "WARNING")
                
            elif message.startswith("HUB_CHECK:"):
                hub_info = message[10:]
                self.log_message(f"Hub check result: {hub_info}", "INFO")
                if "HUB_STATUS:CONNECTED" in hub_info:
                    self.hub_connected = True
                else:
                    self.hub_connected = False
                self.root.after(0, self.update_gui_status)
                
        except Exception as e:
            self.log_message(f"Error parsing response '{message}': {e}", "ERROR")
    
    def update_status_from_dict(self, status_dict):
        try:
            self.log_message(f"Updating status from: {status_dict}", "DEBUG")
            
            if 'HUB' in status_dict:
                self.hub_connected = status_dict['HUB'] == 'C'
                
            if 'BATTERY' in status_dict and status_dict['BATTERY'] != '-1':
                self.battery_level = int(status_dict['BATTERY'])
                
            if 'MOTOR_SPEED' in status_dict:
                self.motor_speed = int(status_dict['MOTOR_SPEED'])
                
            if 'HUB_NAME' in status_dict:
                self.hub_name = status_dict['HUB_NAME']
                
            if 'MOVING' in status_dict:
                self.is_moving = status_dict['MOVING'] == 'Y'
                
            if 'TOTAL_DISTANCE' in status_dict:
                self.total_distance = float(status_dict['TOTAL_DISTANCE'])
                
            if 'SENSOR_DISTANCE' in status_dict:
                self.sensor_distance = float(status_dict['SENSOR_DISTANCE'])
                
            if 'SENSOR_SPEED' in status_dict:
                self.sensor_speed = float(status_dict['SENSOR_SPEED'])
                
            if 'SENSOR_RPM' in status_dict:
                self.sensor_rpm = float(status_dict['SENSOR_RPM'])
                
            if 'TOTAL_TIME' in status_dict:
                self.total_time = int(status_dict['TOTAL_TIME'])
                
            if 'MAX_SPEED' in status_dict:
                self.max_speed = int(status_dict['MAX_SPEED'])
                
            if 'TOTAL_MOVEMENTS' in status_dict:
                self.total_movements = int(status_dict['TOTAL_MOVEMENTS'])
                
            if 'EMERGENCY' in status_dict:
                self.emergency_stop_active = status_dict['EMERGENCY'] == 'Y'
                
            if 'SENSOR_ENABLED' in status_dict:
                self.sensor_enabled = status_dict['SENSOR_ENABLED'] == 'Y'
                
            if 'DISTANCE_MODE' in status_dict:
                self.distance_mode = status_dict['DISTANCE_MODE']
                
            if 'DISTANCE_PROGRESS' in status_dict:
                # Parse distance progress (format: "traveled/target")
                progress_str = status_dict['DISTANCE_PROGRESS']
                if '/' in progress_str:
                    parts = progress_str.split('/')
                    self.distance_traveled = float(parts[0])
                    self.distance_target = float(parts[1])
                    if self.distance_target > 0:
                        self.distance_progress = (self.distance_traveled / self.distance_target) * 100
                
            self.update_gui_status()
            self.update_distance_mode_display()
            self.update_status_display()
            
        except Exception as e:
            self.log_message(f"Error updating status: {e}", "ERROR")
    
    def update_timed_progress(self, progress_dict):
        try:
            if 'ELAPSED' in progress_dict:
                self.timed_elapsed = float(progress_dict['ELAPSED'])
                
            if 'TOTAL' in progress_dict:
                self.timed_duration = float(progress_dict['TOTAL'])
                
            if 'PROGRESS' in progress_dict:
                self.timed_progress = float(progress_dict['PROGRESS'])
                
            # Update progress display if it exists
            if hasattr(self, 'timed_progress_bar'):
                self.timed_progress_bar['value'] = min(self.timed_progress, 100)  # Cap at 100%
                
            if hasattr(self, 'timed_progress_label'):
                distance_info = ""
                if 'DISTANCE' in progress_dict:
                    distance_info = f", Distance: {progress_dict['DISTANCE']} cm"
                    
                self.timed_progress_label.config(
                    text=f"Progress: {self.timed_elapsed:.1f}/{self.timed_duration:.1f} s ({self.timed_progress:.1f}%){distance_info}"
                )
                
        except Exception as e:
            self.log_message(f"Error updating timed progress: {e}", "ERROR")
    
    def handle_timed_completion(self, completion_data):
        try:
            # Reset progress bar
            if hasattr(self, 'timed_progress_bar'):
                self.timed_progress_bar['value'] = 0
                
            if hasattr(self, 'timed_progress_label'):
                self.timed_progress_label.config(text="Progress: Completed ✅")
                
            # Reset after a delay
            self.root.after(3000, self.reset_timed_progress_display)
                
        except Exception as e:
            self.log_message(f"Error handling timed completion: {e}", "ERROR")
    
    def reset_timed_progress_display(self):
        if hasattr(self, 'timed_progress_label'):
            self.timed_progress_label.config(text="Progress: Ready")
    
    def update_distance_progress(self, progress_dict):
        try:
            if 'TRAVELED' in progress_dict:
                self.distance_traveled = float(progress_dict['TRAVELED'])
                
            if 'TARGET' in progress_dict:
                self.distance_target = float(progress_dict['TARGET'])
                
            if 'PROGRESS' in progress_dict:
                self.distance_progress = float(progress_dict['PROGRESS'])
            
            # Extract additional debug information
            start_pos = progress_dict.get('START', 'N/A')
            current_pos = progress_dict.get('CURRENT', 'N/A')
                
            # Update progress display if it exists
            if hasattr(self, 'distance_progress_bar'):
                self.distance_progress_bar['value'] = self.distance_progress
                
            if hasattr(self, 'distance_progress_label'):
                progress_text = f"Progress: {self.distance_traveled:.1f}/{self.distance_target:.1f} cm ({self.distance_progress:.1f}%)"
                if start_pos != 'N/A' and current_pos != 'N/A':
                    progress_text += f" | Start: {start_pos} cm, Current: {current_pos} cm"
                self.distance_progress_label.config(text=progress_text)
                
        except Exception as e:
            self.log_message(f"Error updating distance progress: {e}", "ERROR")
    
    def update_distance_mode_display(self):
        if hasattr(self, 'distance_mode_label'):
            if self.distance_mode == "SENSOR":
                self.distance_mode_label.config(text="🎯 Sensor Feedback", foreground='green')
            else:
                self.distance_mode_label.config(text="⏱️ Time Estimation", foreground='orange')
    
    def toggle_distance_mode(self):
        self.send_command("TOGGLE_DISTANCE_MODE")
        
    def get_distance_mode(self):
        self.send_command("GET_DISTANCE_MODE")
    
    def update_sensor_from_dict(self, sensor_dict):
        try:
            if 'DISTANCE' in sensor_dict:
                self.sensor_distance = float(sensor_dict['DISTANCE'])
                
            if 'SPEED' in sensor_dict:
                self.sensor_speed = float(sensor_dict['SPEED'])
                
            if 'RPM' in sensor_dict:
                self.sensor_rpm = float(sensor_dict['RPM'])
            
            # Update live indicator
            if hasattr(self, 'live_indicator'):
                self.live_indicator.config(text="🟢 Live", foreground='green')
                
            # Update timestamp
            if hasattr(self, 'last_update_label'):
                current_time = datetime.now().strftime('%H:%M:%S.%f')[:-3]  # Include milliseconds
                self.last_update_label.config(text=f"Last update: {current_time}")
            
            # Don't call full update_gui_status for live data - too expensive
            # The fast_gui_update_timer will handle the display updates
            
        except Exception as e:
            self.log_message(f"Error updating sensor data: {e}", "ERROR")
    
    def update_sensor_display_only(self):
        """Update only sensor displays for fast refresh"""
        try:
            # Update sensor labels in control tab
            if hasattr(self, 'sensor_speed_label'):
                self.sensor_speed_label.config(text=f"{self.sensor_speed:.1f} cm/s")
            if hasattr(self, 'sensor_distance_label'):
                self.sensor_distance_label.config(text=f"{self.sensor_distance:.1f} cm")
            if hasattr(self, 'sensor_rpm_label'):
                self.sensor_rpm_label.config(text=f"{self.sensor_rpm:.1f} RPM")
            
            # Update sensor tab display
            if hasattr(self, 'display_distance_label'):
                self.display_distance_label.config(text=f"{self.sensor_distance:.2f} cm")
                self.display_speed_label.config(text=f"{self.sensor_speed:.2f} cm/s")
                self.display_rpm_label.config(text=f"{self.sensor_rpm:.1f} RPM")
            
            # Update statistics comparison
            if hasattr(self, 'sensor_total_distance_label'):
                self.sensor_total_distance_label.config(text=f"{self.sensor_distance:.1f} cm")
                
                # Calculate and update difference
                distance_diff = abs(self.total_distance - self.sensor_distance)
                if hasattr(self, 'distance_diff_label'):
                    self.distance_diff_label.config(text=f"{distance_diff:.1f} cm")
        except Exception as e:
            pass  # Ignore errors during GUI updates
    
    def update_gui_status(self):
        # Update connection status
        if self.connected and self.hub_connected:
            self.status_bar.config(text=f"Fully Connected: ESP32 + LEGO Hub + Sensor | Battery: {self.battery_level}%")
            self.conn_status_label.config(text="ESP32: Connected ✅")
            self.hub_status_label.config(text="LEGO Hub: Connected ✅")
        elif self.connected:
            self.status_bar.config(text="Partially Connected: ESP32 only (Hub disconnected)")
            self.conn_status_label.config(text="ESP32: Connected ✅")
            self.hub_status_label.config(text="LEGO Hub: Disconnected ❌")
        else:
            self.status_bar.config(text="Not Connected")
            self.conn_status_label.config(text="ESP32: Disconnected ❌")
            self.hub_status_label.config(text="LEGO Hub: Disconnected ❌")
        
        # Update sensor status
        if self.connected:
            self.sensor_status_label.config(text=f"Rotation Sensor: {'Enabled' if self.sensor_enabled else 'Disabled'} ✅")
        else:
            self.sensor_status_label.config(text="Rotation Sensor: Unknown ❌")
        
        # Update control tab
        self.current_speed_label.config(text=str(self.motor_speed))
        self.movement_label.config(text="Moving ▶️" if self.is_moving else "Stopped ⏹️")
        self.battery_label.config(text=f"{self.battery_level}%" if self.battery_level >= 0 else "Unknown")
        self.hub_name_label.config(text=self.hub_name if self.hub_name else "Not Connected")
        
        # Update sensor labels in control tab
        self.sensor_speed_label.config(text=f"{self.sensor_speed:.1f} cm/s")
        self.sensor_distance_label.config(text=f"{self.sensor_distance:.1f} cm")
        
        # Update RPM in basic control tab
        if hasattr(self, 'sensor_rpm_label'):
            self.sensor_rpm_label.config(text=f"{self.sensor_rpm:.1f} RPM")
        
        # Update sensor tab display
        if hasattr(self, 'display_distance_label'):
            self.display_distance_label.config(text=f"{self.sensor_distance:.2f} cm")
            self.display_speed_label.config(text=f"{self.sensor_speed:.2f} cm/s")
            self.display_rpm_label.config(text=f"{self.sensor_rpm:.1f} RPM")
        
        # Update statistics with comparison
        self.total_distance_label.config(text=f"{self.total_distance:.1f} cm")
        self.sensor_total_distance_label.config(text=f"{self.sensor_distance:.1f} cm")
        
        # Calculate difference
        distance_diff = abs(self.total_distance - self.sensor_distance)
        self.distance_diff_label.config(text=f"{distance_diff:.1f} cm")
        
        self.total_time_label.config(text=f"{self.total_time} s")
        self.max_speed_label.config(text=str(self.max_speed))
        self.total_movements_label.config(text=str(self.total_movements))
        
        # Update emergency status
        self.update_emergency_status()
        
        # Update speed slider
        if hasattr(self, 'speed_var'):
            self.speed_var.set(self.motor_speed)
        
    def update_emergency_status(self):
        if self.emergency_stop_active:
            self.emergency_btn.config(text="🛑 EMERGENCY ACTIVE", bg='darkred')
            self.clear_emergency_btn.config(state=tk.NORMAL)
            self.emergency_status_label.config(text="Emergency Stop: ACTIVE ⚠️", foreground='red')
            self.quick_clear_btn.config(state=tk.NORMAL)
        else:
            self.emergency_btn.config(text="🛑 EMERGENCY STOP", bg='red')
            self.clear_emergency_btn.config(state=tk.DISABLED)
            self.emergency_status_label.config(text="Emergency Stop: Clear ✅", foreground='green')
            self.quick_clear_btn.config(state=tk.DISABLED)
    
    # BLE Functions - (keeping existing ones and adding sensor functions)
    def scan_for_esp32(self):
        self.log_message("Scanning for ESP32 devices...")
        self.scan_btn.config(state=tk.DISABLED, text="🔍 Scanning...")
        self.connect_btn.config(state=tk.DISABLED)
        for item in self.device_tree.get_children():
            self.device_tree.delete(item)
        threading.Thread(target=self.run_async, args=(self.scan_devices(),), daemon=True).start()
        
    def run_async(self, coro):
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future.result()
        
    async def scan_devices(self):
        try:
            for item in self.device_tree.get_children():
                self.device_tree.delete(item)
                
            devices = await BleakScanner.discover(timeout=10.0)
            esp32_devices = []
            
            for device in devices:
                name = device.name if device.name else "Unknown Device"
                if any(keyword in name.upper() for keyword in ["ESP32", "LEGO"]):
                    esp32_devices.append(device)
                    
            self.root.after(0, lambda: self.update_device_list(esp32_devices))
            
            if esp32_devices:
                self.log_message(f"Found {len(esp32_devices)} ESP32-like devices")
            else:
                self.log_message("No ESP32 devices found")
                
        except Exception as e:
            self.log_message(f"Scan failed: {e}", "ERROR")
        finally:
            self.root.after(0, lambda: self.scan_btn.config(state=tk.NORMAL, text="🔍 Scan for ESP32"))
            
    def update_device_list(self, devices):
        for device in devices:
            name = device.name if device.name else "Unknown Device"
            rssi = getattr(device, 'rssi', 'N/A')
            item = self.device_tree.insert('', 'end', text=name, values=(device.address, rssi))
            
            if "ESP32-LEGO" in name:
                self.device_tree.selection_set(item)
                self.esp32_address = device.address
                self.connect_btn.config(state=tk.NORMAL)
    
    def on_device_selected(self, event):
        selection = self.device_tree.selection()
        if selection:
            item = selection[0]
            device_values = self.device_tree.item(item)['values']
            if device_values:
                self.esp32_address = device_values[0]
                self.connect_btn.config(state=tk.NORMAL)
                device_name = self.device_tree.item(item)['text']
                self.log_message(f"Selected device: {device_name} ({self.esp32_address})")
        else:
            self.connect_btn.config(state=tk.DISABLED)
            self.esp32_address = None
                
    def connect_to_esp32(self):
        selection = self.device_tree.selection()
        if selection:
            item = selection[0]
            device_values = self.device_tree.item(item)['values']
            if device_values:
                self.esp32_address = device_values[0]
                device_name = self.device_tree.item(item)['text']
                self.log_message(f"Using selected device: {device_name} ({self.esp32_address})")
        
        if self.esp32_address:
            self.log_message(f"Connecting to {self.esp32_address}...")
            self.connect_btn.config(state=tk.DISABLED, text="Connecting...")
            threading.Thread(target=self.run_async, args=(self.connect_ble(),), daemon=True).start()
        else:
            messagebox.showwarning("No Device Selected", "Please scan for devices and select one from the list first")
            self.log_message("Connection failed: No device selected")
            
    async def connect_ble(self):
        try:
            self.client = BleakClient(self.esp32_address)
            await self.client.connect()
            
            if self.client.is_connected:
                await self.client.start_notify(CHAR_UUID, self.notification_handler)
                self.connected = True
                self.last_ping_time = time.time()
                self.connection_lost = False
                
                self.root.after(0, self.on_connected)
                self.log_message("Connected successfully!")
                
                await self.send_command_async("INIT_SYSTEM")
                await asyncio.sleep(0.5)
                await self.send_command_async("GET_STATUS")
                await asyncio.sleep(0.5)
                await self.send_command_async("GET_SENSOR_DATA")
                await asyncio.sleep(0.5)
                await self.send_command_async("GET_DISTANCE_MODE")
                
                self.root.after(1000, self.post_connection_checks)
                
            else:
                raise Exception("Failed to establish connection")
                
        except Exception as e:
            self.log_message(f"Connection failed: {e}", "ERROR")
            self.root.after(0, lambda: self.connect_btn.config(state=tk.NORMAL, text="🔗 Connect"))
            
    def on_connected(self):
        self.connect_btn.config(state=tk.DISABLED, text="🔗 Connect")
        self.disconnect_btn.config(state=tk.NORMAL)
        self.scan_btn.config(state=tk.DISABLED)
        
        self.log_message("Connected! Requesting immediate status update...", "INFO")
        self.root.after(500, lambda: self.send_command("GET_STATUS"))
        self.root.after(1000, lambda: self.send_command("GET_SENSOR_DATA"))
        self.root.after(1200, lambda: self.send_command("GET_DISTANCE_MODE"))
        self.root.after(1500, self.update_status_display)
        self.update_gui_status()
        
    def on_disconnected(self):
        self.connected = False
        self.hub_connected = False
        self.connection_lost = False
        self.motor_speed = 0
        self.is_moving = False
        self.battery_level = -1
        self.hub_name = ""
        self.rssi = 0
        self.sensor_distance = 0.0
        self.sensor_speed = 0.0
        self.sensor_rpm = 0.0
        self.last_sensor_update_time = 0
        
        # Reset distance movement tracking
        self.distance_target = 0.0
        self.distance_traveled = 0.0
        self.distance_progress = 0.0
        
        # Reset timed movement tracking
        self.timed_duration = 0.0
        self.timed_elapsed = 0.0
        self.timed_progress = 0.0
        
        # Reset progress bars
        if hasattr(self, 'distance_progress_bar'):
            self.distance_progress_bar['value'] = 0
        if hasattr(self, 'distance_progress_label'):
            self.distance_progress_label.config(text="Progress: Ready")
        if hasattr(self, 'timed_progress_bar'):
            self.timed_progress_bar['value'] = 0
        if hasattr(self, 'timed_progress_label'):
            self.timed_progress_label.config(text="Progress: Ready")
        
        self.connect_btn.config(state=tk.NORMAL, text="🔗 Connect")
        self.disconnect_btn.config(state=tk.DISABLED)
        self.scan_btn.config(state=tk.NORMAL)
        self.update_gui_status()
        self.log_message("Disconnected from ESP32", "INFO")
        
    def disconnect_from_esp32(self):
        if self.client and self.client.is_connected:
            threading.Thread(target=self.run_async, args=(self.disconnect_ble(),), daemon=True).start()
            
    async def disconnect_ble(self):
        try:
            if self.client:
                await self.client.stop_notify(CHAR_UUID)
                await self.client.disconnect()
            self.log_message("Disconnected from ESP32")
        except Exception as e:
            self.log_message(f"Disconnect error: {e}", "ERROR")
        finally:
            self.root.after(0, self.on_disconnected)
    
    # Command Functions
    def send_command(self, command):
        if self.connected:
            threading.Thread(target=self.run_async, args=(self.send_command_async(command),), daemon=True).start()
        else:
            messagebox.showwarning("Not Connected", "Please connect to ESP32 first")
            
    async def send_command_async(self, command):
        try:
            if self.client and self.client.is_connected:
                await self.client.write_gatt_char(CHAR_UUID, command.encode())
                self.log_message(f"Sent: {command}", "SEND")
            else:
                self.log_message("Cannot send command - not connected", "ERROR")
        except Exception as e:
            self.log_message(f"Send command failed: {e}", "ERROR")
    
    # Sensor Functions
    def get_sensor_data(self):
        self.send_command("GET_SENSOR_DATA")
        
    def reset_sensor(self):
        self.send_command("RESET_SENSOR")
        
    def enable_sensor(self):
        self.send_command("SENSOR_ENABLE")
        
    def disable_sensor(self):
        self.send_command("SENSOR_DISABLE")
        
    def debug_distance(self):
        self.send_command("DEBUG_DISTANCE")
    
    # Control Functions (keeping existing ones)
    def on_speed_change(self, value):
        speed = int(float(value))
        self.speed_label.config(text=str(speed))
        
    def set_speed(self, speed):
        self.speed_var.set(speed)
        self.send_command(f"MOTOR_SPEED_{speed}")
        
    def adjust_speed(self, delta):
        current_speed = self.speed_var.get()
        new_speed = max(-100, min(100, current_speed + delta))
        self.speed_var.set(new_speed)
        self.log_message(f"Speed adjusted by {delta}: {current_speed} -> {new_speed}")
        self.send_command(f"MOTOR_SPEED_{new_speed}")
        
    def stop_motor(self):
        self.speed_var.set(0)
        self.send_command("MOTOR_SPEED_0")
        
    def start_timed_movement(self):
        time_ms = self.time_var.get() * 1000
        speed = self.movement_speed_var.get()
        command = f"MOVE_TIME_{time_ms}_SPEED_{speed}"
        self.send_command(command)
        
    def start_distance_movement(self):
        distance = self.distance_var.get()
        speed = self.movement_speed_var.get()
        command = f"MOVE_DISTANCE_{distance}_SPEED_{speed}"
        self.send_command(command)
        
    def emergency_stop(self):
        self.send_command("EMERGENCY_STOP")
        
    def clear_emergency(self):
        self.send_command("CLEAR_EMERGENCY")
        
    def reset_statistics(self):
        self.send_command("RESET_STATS")
        
    def refresh_status(self):
        self.send_command("GET_STATUS")
        self.root.after(200, lambda: self.send_command("GET_SENSOR_DATA"))
        self.root.after(500, self.update_status_display)
        
    def get_battery(self):
        self.send_command("GET_BATTERY")
        
    def test_communication(self):
        self.log_message("Testing communication...", "INFO")
        self.send_command("PING")
        self.root.after(1000, lambda: self.send_command("GET_STATUS"))
        self.root.after(2000, lambda: self.send_command("GET_SENSOR_DATA"))
        self.root.after(3000, self.update_status_display)
        
    def disconnect_hub(self):
        self.send_command("DISCONNECT_HUB")
        
    def reconnect_hub(self):
        self.send_command("RECONNECT_HUB")
        self.log_message("Hub reconnection initiated - please wait 10 seconds", "INFO")
        self.root.after(10000, lambda: self.send_command("GET_STATUS"))
        
    def force_hub_check(self):
        self.log_message("Forcing hub connection check...", "INFO")
        self.send_command("FORCE_HUB_CHECK")
        self.root.after(1000, lambda: self.send_command("GET_STATUS"))
        self.root.after(3000, self.update_status_display)
        
    def post_connection_checks(self):
        self.log_message("Performing post-connection checks for hub status...", "INFO")
        
        def check_sequence():
            self.send_command("GET_STATUS")
            
        delays = [1000, 3000, 5000, 10000, 15000, 20000, 30000]
        for delay in delays:
            self.root.after(delay, check_sequence)
            
        self.root.after(31000, self.update_status_display)
        
    def clear_log(self):
        if hasattr(self, 'log_text'):
            self.log_text.delete(1.0, tk.END)
        
    def save_log(self):
        try:
            from tkinter import filedialog
            filename = filedialog.asksaveasfilename(
                defaultextension=".txt",
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
            )
            if filename and hasattr(self, 'log_text'):
                with open(filename, 'w') as f:
                    f.write(self.log_text.get(1.0, tk.END))
                self.log_message(f"Log saved to {filename}")
        except Exception as e:
            self.log_message(f"Error saving log: {e}", "ERROR")
    
    def auto_refresh_timer(self):
        if self.auto_refresh_var.get() and self.connected:
            self.log_message("Auto-refresh: Requesting status...", "DEBUG")
            self.refresh_status()
        self.root.after(10000, self.auto_refresh_timer)
        
    def connection_monitor_timer(self):
        current_time = time.time()
        
        if self.connected:
            if current_time - self.last_ping_time > 15:
                if not self.connection_lost:
                    self.log_message("Connection timeout detected - ESP32 may have disconnected", "WARNING")
                    self.connection_lost = True
                    threading.Thread(target=self.check_connection_status, daemon=True).start()
            
            elif current_time - self.last_ping_time > self.ping_interval:
                self.send_command("PING")
        
        self.root.after(3000, self.connection_monitor_timer)
    
    def check_connection_status(self):
        try:
            if self.client and not self.client.is_connected:
                self.log_message("BLE connection lost - ESP32 disconnected", "ERROR")
                self.root.after(0, self.on_disconnected)
        except Exception as e:
            self.log_message(f"Connection check failed: {e}", "ERROR")
            self.root.after(0, self.on_disconnected)
        
    def update_status_display(self):
        if not hasattr(self, 'status_text'):
            return
            
        status_info = f"""Enhanced System Status - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
{'='*70}

🔗 CONNECTION STATUS:
  ESP32 Bridge: {'✅ Connected' if self.connected else '❌ Disconnected'}
  LEGO Hub: {'✅ Connected' if self.hub_connected else '❌ Disconnected'}
  Hub Name: {self.hub_name if self.hub_name else 'N/A'}
  Rotation Sensor: {'✅ Enabled' if self.sensor_enabled else '❌ Disabled'} (Pin 2)
  Connection Quality: {'🟢 Excellent' if self.connected and self.hub_connected else '🟡 Partial' if self.connected else '🔴 No Connection'}

🔋 POWER STATUS:
  Battery Level: {self.battery_level if self.battery_level >= 0 else 'Unknown'}%
  RSSI Signal: {self.rssi if self.rssi != 0 else 'N/A'} dB

🚂 MOTOR STATUS:
  Commanded Speed: {self.motor_speed} (-100 to +100)
  Moving: {'✅ Yes' if self.is_moving else '⏹️ No'}
  Emergency Stop: {'🚨 ACTIVE' if self.emergency_stop_active else '✅ Clear'}

🎯 ROTATION SENSOR DATA:
  Distance Traveled: {self.sensor_distance:.2f} cm
  Current Speed: {self.sensor_speed:.2f} cm/s
  Wheel RPM: {self.sensor_rpm:.1f} RPM
  Sensor Status: {'✅ Active' if self.sensor_enabled else '❌ Disabled'}

🏁 MOVEMENT CONTROL:
  Distance Mode: {'🎯 Sensor Feedback' if self.distance_mode == 'SENSOR' else '⏱️ Time Estimation'}
  Active Distance Movement: {'✅ Yes' if hasattr(self, 'distance_target') and self.distance_target > 0 and self.is_moving else '❌ No'}
  Distance Progress: {f'{self.distance_traveled:.1f}/{self.distance_target:.1f} cm ({self.distance_progress:.1f}%)' if hasattr(self, 'distance_target') and self.distance_target > 0 else 'N/A'}
  Active Timed Movement: {'✅ Yes' if hasattr(self, 'timed_duration') and self.timed_duration > 0 and self.is_moving else '❌ No'}
  Timed Progress: {f'{self.timed_elapsed:.1f}/{self.timed_duration:.1f} s ({self.timed_progress:.1f}%)' if hasattr(self, 'timed_duration') and self.timed_duration > 0 else 'N/A'}

📊 MOVEMENT STATISTICS (Estimated vs Actual):
  Estimated Distance: {self.total_distance:.2f} cm
  Actual Distance: {self.sensor_distance:.2f} cm
  Difference: {abs(self.total_distance - self.sensor_distance):.2f} cm
  Total Time: {self.total_time} seconds
  Maximum Speed Used: {self.max_speed}
  Total Movements: {self.total_movements}

💡 AVAILABLE COMMANDS:
  Basic Motor Control:
    • MOTOR_SPEED_<-100 to 100> (e.g., MOTOR_SPEED_50)
    • EMERGENCY_STOP / CLEAR_EMERGENCY
  
  Advanced Movement:
    • MOVE_TIME_<milliseconds>_SPEED_<speed> (e.g., MOVE_TIME_5000_SPEED_60)
      → Runs for exact time with real-time progress and distance tracking
    • MOVE_DISTANCE_<cm>_SPEED_<speed> (e.g., MOVE_DISTANCE_50_SPEED_40)
      → Uses sensor feedback for precise distance control
    • TOGGLE_DISTANCE_MODE (switch between sensor/time-based distance control)
    • GET_DISTANCE_MODE (get current distance control mode)
  
  Sensor Control:
    • GET_SENSOR_DATA / RESET_SENSOR
    • SENSOR_ENABLE / SENSOR_DISABLE
  
  LED Control:
    • HUB_LED_<RED|GREEN|BLUE|YELLOW|PURPLE|WHITE|OFF>
    • ESP32_LED_<RED|GREEN|BLUE|YELLOW|PURPLE|CYAN|WHITE|OFF|BLINK>
  
  System Control:
    • GET_STATUS / GET_BATTERY
    • DISCONNECT_HUB / RECONNECT_HUB
    • RESET_STATS

Last Updated: {datetime.now().strftime('%H:%M:%S')}
        """
        
        self.status_text.delete(1.0, tk.END)
        self.status_text.insert(1.0, status_info)
        self.log_message("Status display updated", "DEBUG")
        
    def run(self):
        self.log_message("Enhanced LEGO Train Controller with Rotation Sensor started")
        self.log_message("Ready to connect to ESP32 bridge")
        self.root.mainloop()

if __name__ == "__main__":
    app = EnhancedTrainController()
    app.run()