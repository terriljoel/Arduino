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

class AdvancedTrainController:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Advanced LEGO Train Controller v2.0")
        self.root.geometry("1000x700")
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
        self.create_led_tab()
        self.create_status_tab()
        self.create_log_tab()
        
        # Status bar at bottom
        self.status_bar = ttk.Label(self.root, text="Ready - Not Connected", 
                                   relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Emergency stop button (always visible)
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
        
        # Quick control buttons
        quick_frame = ttk.Frame(speed_frame)
        quick_frame.pack(fill=tk.X, padx=10, pady=5)
        
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
        
        # Current status
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
        
    def create_advanced_tab(self):
        # Advanced Control Tab
        advanced_tab = ttk.Frame(self.notebook)
        self.notebook.add(advanced_tab, text="⚙️ Advanced")
        
        # Timed movement
        time_frame = ttk.LabelFrame(advanced_tab, text="Timed Movement")
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
        
        # Distance movement
        distance_frame = ttk.LabelFrame(advanced_tab, text="Distance-Based Movement")
        distance_frame.pack(fill=tk.X, padx=10, pady=10)
        
        distance_controls = ttk.Frame(distance_frame)
        distance_controls.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(distance_controls, text="Distance (cm):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        distance_spin = ttk.Spinbox(distance_controls, from_=1, to=1000, textvariable=self.distance_var, width=10, increment=10)
        distance_spin.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(distance_controls, text="Speed:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=5)
        distance_speed_spin = ttk.Spinbox(distance_controls, from_=-100, to=100, textvariable=self.movement_speed_var, width=10)
        distance_speed_spin.grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Button(distance_controls, text="📏 Start Distance Movement", command=self.start_distance_movement).grid(row=0, column=4, padx=10, pady=5)
        
        # Statistics
        stats_frame = ttk.LabelFrame(advanced_tab, text="Movement Statistics")
        stats_frame.pack(fill=tk.X, padx=10, pady=10)
        
        stats_grid = ttk.Frame(stats_frame)
        stats_grid.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(stats_grid, text="Total Distance:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.total_distance_label = ttk.Label(stats_grid, text="0.0 cm", font=('Arial', 10, 'bold'))
        self.total_distance_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(stats_grid, text="Total Time:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        self.total_time_label = ttk.Label(stats_grid, text="0 s", font=('Arial', 10, 'bold'))
        self.total_time_label.grid(row=0, column=3, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(stats_grid, text="Max Speed Used:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.max_speed_label = ttk.Label(stats_grid, text="0", font=('Arial', 10, 'bold'))
        self.max_speed_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)
        
        ttk.Label(stats_grid, text="Total Movements:").grid(row=1, column=2, sticky=tk.W, padx=5, pady=2)
        self.total_movements_label = ttk.Label(stats_grid, text="0", font=('Arial', 10, 'bold'))
        self.total_movements_label.grid(row=1, column=3, sticky=tk.W, padx=5, pady=2)
        
        ttk.Button(stats_frame, text="🔄 Reset Statistics", command=self.reset_statistics).pack(pady=5)
        
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
                 ("Auto Status", "ESP32_LED_AUTO"), ("🧪 Test Pins", "ESP32_LED_TEST")]
        
        for i, (text, cmd) in enumerate(colors):
            ttk.Button(esp32_buttons, text=text, command=lambda c=cmd: self.send_command(c)).grid(row=i//4, column=i%4, padx=5, pady=5)
        
        # LED info
        led_info = ttk.Label(esp32_frame, text="""💡 LED Control Logic:
• Colors (Red/Green/etc): Manual override for 10 seconds, then auto
• OFF: Turn off LED and return to automatic status immediately  
• Auto Status: Return to automatic status immediately
• 🧪 Test Pins: Test each LED pin individually to verify correct wiring""", 
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
        ttk.Checkbutton(refresh_frame, text="Auto-refresh (10s)", variable=self.auto_refresh_var).pack(side=tk.LEFT, padx=20)
        
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
        
        # Connection timing help
        timing_frame = ttk.LabelFrame(status_tab, text="Connection Timing Help")
        timing_frame.pack(fill=tk.X, padx=10, pady=10)
        
        timing_text = ttk.Label(timing_frame, text="""💡 If Hub shows as disconnected after connecting:
1. Click 'Force Hub Check' to refresh hub status
2. Click 'Reconnect Hub' to force ESP32 to reconnect to hub
3. Wait 5-10 seconds and click 'Refresh Status'

⚠️ Note: ESP32 needs time to connect to hub after power-on""", 
                               justify=tk.LEFT, wraplength=600)
        timing_text.pack(padx=10, pady=5)
        
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
        
        # Start auto-refresh timer
        self.auto_refresh_timer()
        
        # Start connection monitoring
        self.connection_monitor_timer()
        
    def log_message(self, message, level="INFO"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}\n"
        
        # Color coding for different log levels
        if hasattr(self, 'log_text'):
            self.log_text.insert(tk.END, log_entry)
            self.log_text.see(tk.END)
            
        print(log_entry.strip())  # Also print to console
        
    def notification_handler(self, sender, data):
        try:
            message = data.decode('utf-8')
            self.log_message(f"Received: {message}", "RECV")
            self.parse_esp32_response(message)
        except UnicodeDecodeError:
            # Handle binary data - might be corrupted transmission
            hex_data = ' '.join(f'{b:02x}' for b in data)
            self.log_message(f"Binary data received (possible corruption): {hex_data}", "WARNING")
            # Try to recover by requesting status
            if self.connected:
                self.root.after(1000, lambda: self.send_command("GET_STATUS"))
    
    def parse_esp32_response(self, message):
        try:
            # Update last communication time
            self.last_ping_time = time.time()
            self.connection_lost = False
            
            if message.startswith("STATUS:"):
                self.log_message(f"Parsing status: {message}", "DEBUG")
                # Parse status message
                parts = message[7:].split(',')  # Remove "STATUS:" prefix
                status_dict = {}
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        status_dict[key] = value
                
                self.log_message(f"Status dict: {status_dict}", "DEBUG")
                # Update GUI with status
                self.root.after(0, lambda: self.update_status_from_dict(status_dict))
                
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
                # Heartbeat response
                pass
                
            elif message.startswith("ERROR:EMERGENCY_STOP_ACTIVE"):
                self.emergency_stop_active = True
                self.root.after(0, self.update_emergency_status)
                self.log_message("Movement blocked: Emergency stop is active!", "WARNING")
                
            elif message.startswith("HUB_CHECK:"):
                # Parse hub check response
                hub_info = message[10:]  # Remove "HUB_CHECK:" prefix
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
                self.log_message(f"Hub connected: {self.hub_connected}", "DEBUG")
                
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
                
            if 'TOTAL_TIME' in status_dict:
                self.total_time = int(status_dict['TOTAL_TIME'])
                
            if 'MAX_SPEED' in status_dict:
                self.max_speed = int(status_dict['MAX_SPEED'])
                
            if 'TOTAL_MOVEMENTS' in status_dict:
                self.total_movements = int(status_dict['TOTAL_MOVEMENTS'])
                
            if 'EMERGENCY' in status_dict:
                self.emergency_stop_active = status_dict['EMERGENCY'] == 'Y'
                
            self.update_gui_status()
            # Also update the status display
            self.update_status_display()
            
        except Exception as e:
            self.log_message(f"Error updating status: {e}", "ERROR")
    
    def update_gui_status(self):
        # Update connection status
        if self.connected and self.hub_connected:
            self.status_bar.config(text=f"Fully Connected: ESP32 + LEGO Hub | Battery: {self.battery_level}%")
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
        
        # Update control tab
        self.current_speed_label.config(text=str(self.motor_speed))
        self.movement_label.config(text="Moving ▶️" if self.is_moving else "Stopped ⏹️")
        self.battery_label.config(text=f"{self.battery_level}%" if self.battery_level >= 0 else "Unknown")
        self.hub_name_label.config(text=self.hub_name if self.hub_name else "Not Connected")
        
        # Update statistics
        self.total_distance_label.config(text=f"{self.total_distance:.1f} cm")
        self.total_time_label.config(text=f"{self.total_time} s")
        self.max_speed_label.config(text=str(self.max_speed))
        self.total_movements_label.config(text=str(self.total_movements))
        
        # Update emergency status
        self.update_emergency_status()
        
        # Update speed slider to match current motor speed
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
    
    # BLE Functions
    def scan_for_esp32(self):
        self.log_message("Scanning for ESP32 devices...")
        self.scan_btn.config(state=tk.DISABLED, text="🔍 Scanning...")
        self.connect_btn.config(state=tk.DISABLED)
        # Clear previous selection
        for item in self.device_tree.get_children():
            self.device_tree.delete(item)
        threading.Thread(target=self.run_async, args=(self.scan_devices(),), daemon=True).start()
        
    def run_async(self, coro):
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future.result()
        
    async def scan_devices(self):
        try:
            # Clear previous devices
            for item in self.device_tree.get_children():
                self.device_tree.delete(item)
                
            devices = await BleakScanner.discover(timeout=10.0)
            esp32_devices = []
            
            for device in devices:
                name = device.name if device.name else "Unknown Device"
                if any(keyword in name.upper() for keyword in ["ESP32", "LEGO"]):
                    esp32_devices.append(device)
                    
            # Update GUI
            self.root.after(0, lambda: self.update_device_list(esp32_devices))
            
            if esp32_devices:
                self.log_message(f"Found {len(esp32_devices)} ESP32-like devices")
            else:
                self.log_message("No ESP32 devices found")
                
        except Exception as e:
            self.log_message(f"Scan failed: {e}", "ERROR")
        finally:
            # Reset scan button state
            self.root.after(0, lambda: self.scan_btn.config(state=tk.NORMAL, text="🔍 Scan for ESP32"))
            
    def update_device_list(self, devices):
        for device in devices:
            name = device.name if device.name else "Unknown Device"
            rssi = getattr(device, 'rssi', 'N/A')
            item = self.device_tree.insert('', 'end', text=name, values=(device.address, rssi))
            
            # Auto-select ESP32-LEGO devices
            if "ESP32-LEGO" in name:
                self.device_tree.selection_set(item)
                self.esp32_address = device.address
                self.connect_btn.config(state=tk.NORMAL)
    
    def on_device_selected(self, event):
        """Handle device selection from the tree"""
        selection = self.device_tree.selection()
        if selection:
            item = selection[0]
            device_values = self.device_tree.item(item)['values']
            if device_values:
                self.esp32_address = device_values[0]  # Address is in first column
                self.connect_btn.config(state=tk.NORMAL)
                device_name = self.device_tree.item(item)['text']
                self.log_message(f"Selected device: {device_name} ({self.esp32_address})")
        else:
            self.connect_btn.config(state=tk.DISABLED)
            self.esp32_address = None
                
    def connect_to_esp32(self):
        # First check if we have a selected device from the tree
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
                
                # Update GUI
                self.root.after(0, self.on_connected)
                self.log_message("Connected successfully!")
                
                # Request initial status and initialize system
                await self.send_command_async("INIT_SYSTEM")
                await asyncio.sleep(0.5)
                await self.send_command_async("GET_STATUS")
                await asyncio.sleep(0.5)
                
                # Schedule multiple status checks to catch hub connection
                self.root.after(1000, self.post_connection_checks)
                
            else:
                raise Exception("Failed to establish connection")
                
        except Exception as e:
            self.log_message(f"Connection failed: {e}", "ERROR")
            # Reset button state on failure
            self.root.after(0, lambda: self.connect_btn.config(state=tk.NORMAL, text="🔗 Connect"))
            
    def on_connected(self):
        self.connect_btn.config(state=tk.DISABLED, text="🔗 Connect")
        self.disconnect_btn.config(state=tk.NORMAL)
        self.scan_btn.config(state=tk.DISABLED)
        
        # Force immediate status update
        self.log_message("Connected! Requesting immediate status update...", "INFO")
        self.root.after(500, lambda: self.send_command("GET_STATUS"))
        self.root.after(1500, self.update_status_display)
        self.update_gui_status()
        
    def on_disconnected(self):
        self.connected = False
        self.hub_connected = False
        self.connection_lost = False
        # Reset all status variables
        self.motor_speed = 0
        self.is_moving = False
        self.battery_level = -1
        self.hub_name = ""
        self.rssi = 0
        
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
    
    # Control Functions
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
        # Also manually update the status display
        self.root.after(500, self.update_status_display)  # Update display after 500ms delay
        
    def get_battery(self):
        self.send_command("GET_BATTERY")
        
    def test_communication(self):
        """Test basic communication with ESP32"""
        self.log_message("Testing communication...", "INFO")
        self.send_command("PING")
        self.root.after(1000, lambda: self.send_command("GET_STATUS"))
        self.root.after(2000, self.update_status_display)
        
    def disconnect_hub(self):
        self.send_command("DISCONNECT_HUB")
        
    def reconnect_hub(self):
        self.send_command("RECONNECT_HUB")
        self.log_message("Hub reconnection initiated - please wait 10 seconds", "INFO")
        # Check status after delay to see if hub connected
        self.root.after(10000, lambda: self.send_command("GET_STATUS"))
        
    def force_hub_check(self):
        """Force check hub connection status"""
        self.log_message("Forcing hub connection check...", "INFO")
        self.send_command("FORCE_HUB_CHECK")
        self.root.after(1000, lambda: self.send_command("GET_STATUS"))
        self.root.after(3000, self.update_status_display)
        
    def post_connection_checks(self):
        """Perform multiple status checks after initial connection to catch hub connection"""
        self.log_message("Performing post-connection checks for hub status...", "INFO")
        
        def check_sequence():
            self.send_command("GET_STATUS")
            
        # Check status multiple times over 30 seconds to catch hub connection
        delays = [1000, 3000, 5000, 10000, 15000, 20000, 30000]  # milliseconds
        for delay in delays:
            self.root.after(delay, check_sequence)
            
        # Update display after final check
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
        self.root.after(10000, self.auto_refresh_timer)  # Every 10 seconds
        
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
        
    def update_status_display(self):
        if not hasattr(self, 'status_text'):
            return
            
        status_info = f"""System Status - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
{'='*60}

🔗 CONNECTION STATUS:
  ESP32 Bridge: {'✅ Connected' if self.connected else '❌ Disconnected'}
  LEGO Hub: {'✅ Connected' if self.hub_connected else '❌ Disconnected'}
  Hub Name: {self.hub_name if self.hub_name else 'N/A'}
  Connection Quality: {'🟢 Excellent' if self.connected and self.hub_connected else '🟡 Partial' if self.connected else '🔴 No Connection'}

🔋 POWER STATUS:
  Battery Level: {self.battery_level if self.battery_level >= 0 else 'Unknown'}%
  RSSI Signal: {self.rssi if self.rssi != 0 else 'N/A'} dB

🚂 MOTOR STATUS:
  Current Speed: {self.motor_speed} (-100 to +100)
  Moving: {'✅ Yes' if self.is_moving else '⏹️ No'}
  Emergency Stop: {'🚨 ACTIVE' if self.emergency_stop_active else '✅ Clear'}

📊 MOVEMENT STATISTICS:
  Total Distance: {self.total_distance:.2f} cm
  Total Time: {self.total_time} seconds
  Maximum Speed Used: {self.max_speed}
  Total Movements: {self.total_movements}

💡 AVAILABLE COMMANDS:
  Basic Motor Control:
    • MOTOR_SPEED_<-100 to 100> (e.g., MOTOR_SPEED_50)
    • EMERGENCY_STOP / CLEAR_EMERGENCY
  
  Advanced Movement:
    • MOVE_TIME_<milliseconds>_SPEED_<speed> (e.g., MOVE_TIME_5000_SPEED_60)
    • MOVE_DISTANCE_<cm>_SPEED_<speed> (e.g., MOVE_DISTANCE_50_SPEED_40)
  
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
        self.log_message("Advanced LEGO Train Controller started")
        self.log_message("Ready to connect to ESP32 bridge")
        self.root.mainloop()

if __name__ == "__main__":
    app = AdvancedTrainController()
    app.run()