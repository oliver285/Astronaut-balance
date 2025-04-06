import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
from three_Tether_Dynamic_Model_eqs import three_teth_model
import sys
import time
import numpy as np
import threading
import random
from encoder_phidget import phidget_encoder
import csv
from datetime import datetime, timezone
import time as time_module
import os

class ThreeTetherGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Three Tether Operation Interface")
        self.root.geometry("800x600")
        
        # Variables
        self.com_port = tk.StringVar()
        self.baud_rate = tk.StringVar(value="115200")
        self.serial_connection = None
        self.running = False
        self.manual_monitoring = False
        self.three_teth_calc = three_teth_model()
        self.encoder_one = phidget_encoder(1)
        self.encoder_two = phidget_encoder(2)
        self.encoder_three = phidget_encoder(0)
        
        # User inputs
        self.motor_count = tk.StringVar()
        self.user_mass = tk.StringVar()
        self.user_height = tk.StringVar()
        self.user_waist = tk.StringVar()
        self.tether_one_length = tk.StringVar()
        self.tether_two_length = tk.StringVar()
        self.tether_three_length = tk.StringVar()
        
        # Encoder offsets
        self.encoder_one_offset = 0
        self.encoder_two_offset = 0
        self.encoder_three_offset = 0
        
        # Tether lengths initialized flag
        self.tether_lengths_initialized = False
        
        # Data storage
        self.time_utc = []
        # Removed time_microsec and start_time_microsec
        self.tether_length_one = []
        self.tether_length_two = []
        self.tether_length_three = []
        self.apex = []
        self.tether_force_one = []
        self.tether_force_two = []
        self.tether_force_three = []
        self.states = []  # Track the current state for each data point
        self.operation_modes = []  # Track operation mode (automatic/manual)
        self.sent_commands = []  # Track sent commands
        self.response_times = []
        self.loadcell_force_one = []
        self.loadcell_force_two = []
        self.loadcell_force_three = []
        self.responses = []  # Store raw responses
        self.tether_initialized_status = []
        self.force_error = []
        self.angular_error = []
        
        # Initialize command vector
        self.command_vec = []
        self.current_state = 3  # Initialize state to 3
        self.manual_state = 3  # Default state
        self.manual_force1 = 0.0
        self.manual_force2 = 0.0
        self.manual_force3 = 0.0
        self.latest_command = ""
        
        # Create the initial connection UI
        self.create_connection_ui()
    
    def create_connection_ui(self):
        # Connection frame
        connection_frame = ttk.LabelFrame(self.root, text="Serial Connection")
        connection_frame.pack(fill="x", padx=10, pady=10)
        
        # COM port selection
        ttk.Label(connection_frame, text="COM Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        # Get available ports
        ports = [port.device for port in serial.tools.list_ports.comports()]
        com_combo = ttk.Combobox(connection_frame, textvariable=self.com_port, values=ports)
        com_combo.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        # Baud rate
        ttk.Label(connection_frame, text="Baud Rate:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        baud_entry = ttk.Entry(connection_frame, textvariable=self.baud_rate)
        baud_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        # Connect button
        connect_btn = ttk.Button(connection_frame, text="Connect", command=self.connect_serial)
        connect_btn.grid(row=2, column=0, columnspan=2, padx=5, pady=10)
        
        # Configure grid weights
        connection_frame.columnconfigure(1, weight=1)
    
    def create_configuration_ui(self):
        # Clear previous UI
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # Configuration frame - Only number of motors now
        config_frame = ttk.LabelFrame(self.root, text="System Configuration")
        config_frame.pack(fill="x", padx=10, pady=10)
        
        # Number of motors
        ttk.Label(config_frame, text="Number of Motors:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        motor_entry = ttk.Entry(config_frame, textvariable=self.motor_count)
        motor_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        # Confirm button
        confirm_btn = ttk.Button(config_frame, text="Confirm", command=self.confirm_configuration)
        confirm_btn.grid(row=1, column=0, columnspan=2, padx=5, pady=10)
        
        # Configure grid weights
        config_frame.columnconfigure(1, weight=1)
    
    def create_tether_length_ui(self):
        # Clear previous UI
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # Tether lengths frame
        tether_frame = ttk.LabelFrame(self.root, text="Tether Length Configuration")
        tether_frame.pack(fill="x", padx=10, pady=10)
        
        # Tether lengths
        ttk.Label(tether_frame, text="Tether 1 Length (inches):").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        tether1_entry = ttk.Entry(tether_frame, textvariable=self.tether_one_length)
        tether1_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(tether_frame, text="Tether 2 Length (inches):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        tether2_entry = ttk.Entry(tether_frame, textvariable=self.tether_two_length)
        tether2_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(tether_frame, text="Tether 3 Length (inches):").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        tether3_entry = ttk.Entry(tether_frame, textvariable=self.tether_three_length)
        tether3_entry.grid(row=2, column=1, padx=5, pady=5, sticky="ew")
        
        # Two buttons
        button_frame = ttk.Frame(tether_frame)
        button_frame.grid(row=3, column=0, columnspan=2, padx=5, pady=10)
        
        # Skip button
        skip_btn = ttk.Button(button_frame, text="Skip", command=self.skip_tether_initialization)
        skip_btn.pack(side=tk.LEFT, padx=5)
        
        # Initialize button
        init_btn = ttk.Button(button_frame, text="Initialize Tethers", command=self.initialize_tether_lengths)
        init_btn.pack(side=tk.RIGHT, padx=5)
        
        # Configure grid weights
        tether_frame.columnconfigure(1, weight=1)
    
    def create_operation_ui(self):
        # Clear previous UI
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # Operation frame
        operation_frame = ttk.LabelFrame(self.root, text="System Operation")
        operation_frame.pack(fill="x", padx=10, pady=10)
        
        # Add tether initialization button if not already initialized
        if not self.tether_lengths_initialized:
            self.tether_init_btn = ttk.Button(operation_frame, text="Initialize Tether Lengths", 
                                            command=self.create_tether_length_ui)
            self.tether_init_btn.grid(row=0, column=0, columnspan=2, padx=5, pady=5)
        
        # Start/Stop buttons
        self.start_btn = ttk.Button(operation_frame, text="Start", command=self.start_operation)
        self.start_btn.grid(row=1, column=0, padx=5, pady=10)
        
        self.stop_btn = ttk.Button(operation_frame, text="Stop", command=self.stop_operation, state="disabled")
        self.stop_btn.grid(row=1, column=1, padx=5, pady=10)
        
        # State buttons frame
        state_frame = ttk.LabelFrame(self.root, text="States")
        state_frame.pack(fill="x", padx=10, pady=10)
        
        # State buttons
        for i in range(1, 6):
            btn = ttk.Button(state_frame, text=f"State {i}", 
                            command=lambda state=i: self.set_state(state))
            btn.grid(row=0, column=i-1, padx=5, pady=5)
        
        # Manual control frame - Added after state buttons
        manual_frame = ttk.LabelFrame(self.root, text="Manual Control")
        manual_frame.pack(fill="x", padx=10, pady=10)
        
        # Manual control checkbox
        self.manual_control = tk.BooleanVar(value=False)
        manual_check = ttk.Checkbutton(manual_frame, text="Enable Manual Control", 
                                    variable=self.manual_control)
        manual_check.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        # Command entry
        ttk.Label(manual_frame, text="Command:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.command_entry = ttk.Entry(manual_frame, width=50)
        self.command_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        # Send button
        send_btn = ttk.Button(manual_frame, text="Send Command", command=self.send_manual_command)
        send_btn.grid(row=1, column=2, padx=5, pady=5)
        
        # Configure grid weights for manual frame
        manual_frame.columnconfigure(1, weight=1)
        
        # Data display frame - Now after the manual control section
        data_frame = ttk.LabelFrame(self.root, text="Data")
        data_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Scrolled text widget for data display
        self.data_text = tk.Text(data_frame, height=15, width=70)
        scroll = ttk.Scrollbar(data_frame, command=self.data_text.yview)
        self.data_text.configure(yscrollcommand=scroll.set)
        self.data_text.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        scroll.grid(row=0, column=1, sticky="ns")
        
        # Configure grid weights
        data_frame.columnconfigure(0, weight=1)
        data_frame.rowconfigure(0, weight=1)
    
    def connect_serial(self):
        try:
            self.serial_connection = serial.Serial(
                port=self.com_port.get(),
                baudrate=int(self.baud_rate.get()),
                timeout=1
            )
            messagebox.showinfo("Connection Successful", 
                               f"Successfully connected to {self.com_port.get()} at {self.baud_rate.get()} baud")
            self.create_configuration_ui()
        except Exception as e:
            messagebox.showerror("Connection Failed", f"Failed to connect: {str(e)}")
    
    def confirm_configuration(self):
        # Validate motor count input
        try:
            # Basic validation - just make sure it can be converted to a number
            motor_count = int(self.motor_count.get())
            
            # Send the motor count to the microcontroller
            motor_message = self.motor_count.get() + '\n'
        
            self.serial_connection.write(motor_message.encode('utf-8'))

            response = self.serial_connection.readline().decode('utf-8').strip()
            print(response)
            self.serial_connection.reset_input_buffer()
            
            # Go to tether length UI
            self.create_tether_length_ui()
            
        except ValueError:
            messagebox.showerror("Input Error", "Please ensure the motor count is a valid number.")
    
    def skip_tether_initialization(self):
        self.tether_lengths_initialized = False
        # Switch to operation UI
        self.create_operation_ui()
        self.update_data_display("Tether lengths not initialized. Some features will be limited.\n")
    
    def initialize_tether_lengths(self):
        # Validate tether length inputs
        try:
            # Check if all tether lengths are provided
            if not all([self.tether_one_length.get(), self.tether_two_length.get(), self.tether_three_length.get()]):
                messagebox.showerror("Input Error", "Please provide all three tether lengths.")
                return
            
            
            # Convert to float
            tether1 = float(self.tether_one_length.get())
            tether2 = float(self.tether_two_length.get())
            tether3 = float(self.tether_three_length.get())
            
            # Store current encoder readings as offsets
            self.encoder_one_offset = self.encoder_one.get_angle()
            self.encoder_two_offset = self.encoder_two.get_angle()
            self.encoder_three_offset = self.encoder_three.get_angle()
            
            # Set initialized flag
            self.tether_lengths_initialized = True
            
            # Switch to operation UI
            self.create_operation_ui()
            self.update_data_display(
                f"Tether lengths initialized:\n"
                f"Tether 1: {tether1} inches, Offset: {self.encoder_one_offset}\n"
                f"Tether 2: {tether2} inches, Offset: {self.encoder_two_offset}\n"
                f"Tether 3: {tether3} inches, Offset: {self.encoder_three_offset}\n"
            )
            
        except ValueError:
            messagebox.showerror("Input Error", "Please ensure all tether lengths are valid numbers.")
        
    def start_operation(self):
        self.running = True
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        
        # Make sure current_state is set to 3 when starting
        self.current_state = 3
        
        # Start the operation in a separate thread
        self.operation_thread = threading.Thread(target=self.run_operation)
        self.operation_thread.daemon = True
        self.operation_thread.start()
    
    def stop_operation(self):
        self.running = False
        self.manual_monitoring = False 
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        
    def set_state(self, state):
        if self.running:
            self.current_state = state
            self.data_text.insert(tk.END, f"Changed to State {state}\n")
            self.data_text.see(tk.END)
    
    def get_tether_lengths_and_calculate_apex(self, initial_guess):
        # If tether lengths are not initialized, return defaults and skip calculations
        if not self.tether_lengths_initialized:
            return None, None, None, None, [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        
        # Get tether lengths from encoders with offsets
        tether1_length = self.encoder_one.angle2length(
            float(self.tether_one_length.get()), offset=self.encoder_one_offset)
        tether2_length = self.encoder_two.angle2length(
            float(self.tether_two_length.get()), offset=self.encoder_two_offset)
        tether3_length = self.encoder_three.angle2length(
            float(self.tether_three_length.get()), offset=self.encoder_three_offset)
        
        print(tether1_length)
        print(tether2_length)
        print(tether3_length)
        
        # Calculate apex position
        apex_position = self.three_teth_calc.calculate_apex(
            tether1_length/12, tether2_length/12, tether3_length/12, initial_guess
        )
        print(apex_position)
        
        # Calculate forces
        forces = self.three_teth_calc.calculate_tether_forces(apex_position)
        print(forces)
        
        return tether1_length, tether2_length, tether3_length, apex_position, forces
        
    def run_operation(self):
        j = 0
        
        while self.running:
            # Check if we're in manual mode
            if self.manual_control.get():
                # If switched to manual mode, exit the automatic loop
                self.root.after(0, lambda: self.update_data_display("Switched to manual mode\n"))
                break
                
            try:
                # Record timestamp (UTC only now)
                current_utc = datetime.now(timezone.utc)
                
                # Initialize default values
                tether1_length = None
                tether2_length = None
                tether3_length = None
                apex_position = None
                forces = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]  # Default forces
                
                # Calculate tether lengths and apex if initialized
                if self.tether_lengths_initialized:
                    initial_guess = self.apex[-1] if (j > 0 and len(self.apex) > 0 and self.apex[-1] is not None) else [0, 0, -3]
                    tether1_length, tether2_length, tether3_length, apex_position, forces = self.get_tether_lengths_and_calculate_apex(initial_guess)
                
                # Record data
                self.time_utc.append(current_utc)
                self.operation_modes.append("Automatic")
                self.tether_length_one.append(tether1_length)
                self.tether_length_two.append(tether2_length)
                self.tether_length_three.append(tether3_length)
                self.apex.append(apex_position)
                self.tether_initialized_status.append(self.tether_lengths_initialized)
                
                # Use calculated forces if available, otherwise use zeros
                if self.tether_lengths_initialized:
                    self.tether_force_one.append(forces[0][0])
                    self.tether_force_two.append(forces[1][0])
                    self.tether_force_three.append(forces[2][0])
                else:
                    self.tether_force_one.append(0)
                    self.tether_force_two.append(0)
                    self.tether_force_three.append(0)
                
                self.states.append(self.current_state)
                
                # Format command with all required parameters
                command = f'{self.current_state},{self.tether_force_one[-1]:.2f},{self.tether_force_two[-1]:.2f},{self.tether_force_three[-1]:.2f}\n'
                
                # Clear input buffer before sending new command to avoid multiple responses
                self.serial_connection.reset_input_buffer()
                
                # Send command to microcontroller
                self.serial_connection.write(command.encode('utf-8'))
                
                # Keep track of sent commands
                self.command_vec.append(command)
                self.sent_commands.append(command.strip())
                
                # Format the output string with the current state and mode
                output_text = f"Mode: Automatic\n"
                
                if self.tether_lengths_initialized:
                    output_text += f"Apex: {apex_position}\n"
                else:
                    output_text += "Tether lengths not initialized - using default values\n"
                    output_text += f"Forces: 0.0000, 0.0000, 0.0000\n"
                
                output_text += f"Command: {command.strip()}\n"
                
                # Update the UI with the command output
                self.root.after(0, lambda t=output_text: self.update_data_display(t))
                
                # Wait for response (non-blocking, with timeout)
                response = None
                response_timeout = time.time() + 2.0  # 2 second timeout
                
                while response is None and time.time() < response_timeout:
                    if not self.running or self.manual_control.get():
                        # Exit loop if stopped or switched to manual mode
                        return
                        
                    if self.serial_connection.in_waiting > 0:
                        response = self.serial_connection.readline().decode('utf-8').strip()
                        if response:
                            self.process_response(response, j, apex_position)
                            break
                    
                    # Small sleep to prevent CPU hogging
                    time.sleep(0.001)
                
                # If no response was received within timeout
                if response is None:
                    self.update_data_display("No response received within timeout, continuing...\n")
                    # Add placeholder data to ensure consistency
                    self.responses.append("No response")
                    self.response_times.append(None)
                    self.loadcell_force_one.append(None)
                    self.loadcell_force_two.append(None)
                    self.loadcell_force_three.append(None)
                    self.force_error.append(None)
                    self.angular_error.append(None)
                
                # Increment counter
                j += 1
                
                # No additional delay - the loop will continue as soon as a response is received or times out
            
            except Exception as e:
                error_msg = f"Error in operation: {str(e)}\n"
                self.root.after(0, lambda msg=error_msg: self.update_data_display(msg))
                time.sleep(1)  # Short delay after an error
                
                # Add placeholder data to ensure consistency in data arrays
                if len(self.operation_modes) > len(self.loadcell_force_one):
                    self.loadcell_force_one.append(None)
                    self.loadcell_force_two.append(None)
                    self.loadcell_force_three.append(None)
                    self.responses.append("")
                    self.response_times.append(None)
                    self.force_error.append(None)
                    self.angular_error.append(None)

    def process_response(self, response, index, apex=None):
        try:
            # Store raw response
            self.responses.append(response)
            
            # Parse the response (expected format: time,loadcell1,loadcell2,loadcell3)
            parts = response.split(',')
            
            if len(parts) >= 4:
                self.response_times.append(float(parts[0]) if parts[0] else None)
                self.loadcell_force_one.append(float(parts[1]) if parts[1] else None)
                self.loadcell_force_two.append(float(parts[2]) if parts[2] else None)
                self.loadcell_force_three.append(float(parts[3]) if parts[3] else None)
                
            if apex is not None and self.tether_lengths_initialized:
                # Use only the most recent loadcell force values
                loadcell1 = self.loadcell_force_one[-1] if self.loadcell_force_one else None
                loadcell2 = self.loadcell_force_two[-1] if self.loadcell_force_two else None
                loadcell3 = self.loadcell_force_three[-1] if self.loadcell_force_three else None
                
                # Only proceed if we have valid loadcell values
                if loadcell1 is not None and loadcell2 is not None and loadcell3 is not None:
                    f_err, angle_err, _, _, _ = self.three_teth_calc.calculate_tether_error(apex, [loadcell1, loadcell2, loadcell3])
                    self.force_error.append(f_err)
                    self.angular_error.append(angle_err)
                
                # Display response
                response_text = f"Response: {response}\n"
                self.root.after(0, lambda t=response_text: self.update_data_display(t))
            else:
                # Handle non-numeric responses (typical for state 2)
                self.root.after(0, lambda r=response: self.update_data_display(f"Response: {r}\n"))
                self.response_times.append(None)
                self.loadcell_force_one.append(None)
                self.loadcell_force_two.append(None)
                self.loadcell_force_three.append(None)
                self.force_error.append(None)
                self.angular_error.append(None)
        
        except Exception as e:
            error_msg = f"Error processing response: {str(e)}\n"
            self.root.after(0, lambda msg=error_msg: self.update_data_display(msg))
            # Add empty placeholders if parsing fails
            if len(self.responses) <= index:
                self.responses.append(response)
            if len(self.response_times) <= index:
                self.response_times.append(None)
            if len(self.loadcell_force_one) <= index:
                self.loadcell_force_one.append(None)
            if len(self.loadcell_force_two) <= index:
                self.loadcell_force_two.append(None)
            if len(self.loadcell_force_three) <= index:
                self.loadcell_force_three.append(None)
            if len(self.force_error) <= index:
                self.force_error.append(None)
            if len(self.angular_error) <= index:
                self.angular_error.append(None)
        
    def update_data_display(self, text):
        self.data_text.insert(tk.END, text)
        self.data_text.see(tk.END)
    
    def save_data_to_csv(self, custom_filename=None, custom_directory=None):
        try:
            if custom_filename and custom_directory:
                filepath = os.path.join(custom_directory, custom_filename)
                if not filepath.endswith('.csv'):
                    filepath += '.csv'
            else:
                # Default timestamp-based filename
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filepath = f"tether_data_{timestamp}.csv"
            
            with open(filepath, 'w', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                
                # Write header row with additional columns for response data
                # Removed 'Microseconds Since Start' from the header
                csvwriter.writerow([
                    'UTC Time',
                    'Tether 1 Length', 'Tether 2 Length', 'Tether 3 Length',
                    'X Apex', 'Y Apex', 'Z Apex',
                    'Tether 1 Force', 'Tether 2 Force', 'Tether 3 Force',
                    'Current State', 'Operation Mode', 'Sent Command',
                    'Response Time', 'Loadcell 1 Force', 'Loadcell 2 Force', 'Loadcell 3 Force', 'Force Error (lbf)', 'Angle Error (deg)', 'Raw Response',
                    'Tether Lengths Initialized'
                ])
                
                # Write data rows
                for i in range(len(self.time_utc)):
                    # Get values, using None for missing data
                    command = self.sent_commands[i] if i < len(self.sent_commands) else ""
                    mode = self.operation_modes[i] if i < len(self.operation_modes) else "Automatic"
                    
                    # Handle apex data which may be None for state 2 or when tethers aren't initialized
                    if i < len(self.apex) and self.apex[i] is not None:
                        apex_x = self.apex[i][0]
                        apex_y = self.apex[i][1]
                        apex_z = self.apex[i][2]
                    else:
                        apex_x = None
                        apex_y = None
                        apex_z = None
                    
                    response = self.responses[i] if i < len(self.responses) else ""
                    resp_time = self.response_times[i] if i < len(self.response_times) else None
                    lc1 = self.loadcell_force_one[i] if i < len(self.loadcell_force_one) else None
                    lc2 = self.loadcell_force_two[i] if i < len(self.loadcell_force_two) else None
                    lc3 = self.loadcell_force_three[i] if i < len(self.loadcell_force_three) else None
                    
                    # Write the row without microseconds
                    csvwriter.writerow([
                        self.time_utc[i],
                        self.tether_length_one[i] if i < len(self.tether_length_one) else None,
                        self.tether_length_two[i] if i < len(self.tether_length_two) else None,
                        self.tether_length_three[i] if i < len(self.tether_length_three) else None,
                        apex_x,
                        apex_y,
                        apex_z,
                        self.tether_force_one[i] if i < len(self.tether_force_one) else None,
                        self.tether_force_two[i] if i < len(self.tether_force_two) else None,
                        self.tether_force_three[i] if i < len(self.tether_force_three) else None,
                        self.states[i] if i < len(self.states) else None,
                        mode,
                        command,
                        resp_time,
                        lc1,
                        lc2, 
                        lc3,
                        self.force_error[i] if i < len(self.force_error) else None,
                        self.angular_error[i] if i < len(self.angular_error) else None,
                        response,
                        self.tether_initialized_status[i] if i < len(self.tether_initialized_status) else False
                    ])
            
            return filepath  # Return the filepath for confirmation message
        except Exception as e:
            return f"Error saving data: {str(e)}"

    def send_manual_command(self):
        if not self.manual_control.get():
            messagebox.showinfo("Manual Control", "Please enable manual control first")
            return
            
        command = self.command_entry.get().strip()
        if not command:
            messagebox.showinfo("Manual Control", "Please enter a command")
            return
            
        try:
            # Parse command to get state and forces
            command_parts = command.split(',')
            if len(command_parts) >= 4:
                self.manual_state = int(command_parts[0])
                self.manual_force1 = float(command_parts[1])
                self.manual_force2 = float(command_parts[2])
                self.manual_force3 = float(command_parts[3])
            else:
                self.update_data_display("Error: Command must be in format: state,force1,force2,force3\n")
                return
                
            # Check if serial connection is open
            if not self.serial_connection or not self.serial_connection.is_open:
                messagebox.showerror("Connection Error", "Serial connection is not open")
                return
            
            # Ensure command is a string before adding newline
            command_str = command
            # Add newline character if not present
            if not command_str.endswith('\n'):
                command_str += '\n'
                
            # Only store the command information, not tether data yet
            self.latest_command = command.strip()
            self.update_data_display(f"Manual Command Sent: {command.strip()}\n")

            # Send command to microcontroller
            self.serial_connection.write(command_str.encode('utf-8'))
            
            # Start monitoring thread if needed
            if not hasattr(self, 'manual_response_thread') or not self.manual_response_thread.is_alive():
                self.manual_monitoring = True
                self.manual_response_thread = threading.Thread(target=self.monitor_manual_responses)
                self.manual_response_thread.daemon = True
                self.manual_response_thread.start()

            # Clear the command entry
            self.command_entry.delete(0, tk.END)
            
        except Exception as e:
            error_msg = f"Error in manual command: {str(e)}\n"
            self.update_data_display(error_msg)

    def monitor_manual_responses(self):
        while self.manual_control.get() and self.manual_monitoring:
            try:
                if self.serial_connection.in_waiting > 0:
                    response = self.serial_connection.readline().decode('utf-8').strip()
                    if response:
                        # Get the latest command data (from class instance variables)
                        manual_state = self.manual_state
                        manual_force1 = self.manual_force1
                        manual_force2 = self.manual_force2
                        manual_force3 = self.manual_force3
                        
                        # Record timestamp (UTC only now)
                        current_utc = datetime.now(timezone.utc)
                        
                        # Store base data that's always needed
                        self.time_utc.append(current_utc)
                        self.operation_modes.append("Manual")
                        self.states.append(manual_state)
                        self.sent_commands.append(self.latest_command if hasattr(self, 'latest_command') else "")
                        self.tether_initialized_status.append(self.tether_lengths_initialized)
                        
                        # Initialize values
                        tether1_length = None
                        tether2_length = None
                        tether3_length = None
                        apex_position = None
                        
                        # Calculate tether data if initialized
                        if self.tether_lengths_initialized:
                            # For states where we need tether calculations
                            initial_guess = self.apex[-1] if (len(self.apex) > 0 and self.apex[-1] is not None) else [0, 0, -3]
                            tether1_length, tether2_length, tether3_length, apex_position, _ = self.get_tether_lengths_and_calculate_apex(initial_guess)
                            
                            # Store the tether data
                            self.tether_length_one.append(tether1_length)
                            self.tether_length_two.append(tether2_length)
                            self.tether_length_three.append(tether3_length)
                            self.apex.append(apex_position)
                            
                            #Display data
                            self.root.after(0, lambda t1=tether1_length, t2=tether2_length, t3=tether3_length, ap=apex_position, 
                                            f1=manual_force1, f2=manual_force2, f3=manual_force3: 
                                            self.update_data_display(
                                                f"Tether data for State {manual_state}:\n"
                                                f"Lengths: {t1:.2f}, {t2:.2f}, {t3:.2f}\n"
                                                f"Apex: {ap}\n"
                                                f"Forces: {f1}, {f2}, {f3}\n"
                                            ))
                        else:
                            # For other states without tether calculations
                            self.tether_length_one.append(None)
                            self.tether_length_two.append(None)
                            self.tether_length_three.append(None)
                            self.apex.append(None)
                            
                            # Show basic info when tethers not initialized
                            self.root.after(0, lambda st=manual_state: 
                                            self.update_data_display(f"Manual command for State {st} (Tethers not initialized)\n"))
                        
                        # Always store the forces from manual command
                        self.tether_force_one.append(manual_force1)
                        self.tether_force_two.append(manual_force2)
                        self.tether_force_three.append(manual_force3)
                        
                        # Process response
                        self.process_response(response, len(self.time_utc) - 1, apex_position)
                
                # Small sleep to prevent CPU hogging
                time.sleep(0.001)
                
            except Exception as e:
                error_msg = f"Error monitoring responses: {str(e)}\n"
                self.root.after(0, lambda msg=error_msg: self.update_data_display(msg))
                time.sleep(0.1)

    def on_closing(self):
        if self.running:
            self.stop_operation()
        self.manual_monitoring = False
        
        # Ask user if they want to save data
        if len(self.time_utc) > 0:  # Only offer to save if there's data
            save = messagebox.askyesno("Save Data", "Would you like to save the collected data to CSV?")
            if save:
                # Ask for filename
                filename_dialog = tk.Toplevel(self.root)
                filename_dialog.title("Save Data")
                filename_dialog.geometry("400x150")
                filename_dialog.transient(self.root)
                filename_dialog.grab_set()
                
                # Filename entry
                ttk.Label(filename_dialog, text="Enter filename:").grid(row=0, column=0, padx=10, pady=10, sticky="w")
                filename_var = tk.StringVar(value=f"tether_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
                filename_entry = ttk.Entry(filename_dialog, textvariable=filename_var, width=30)
                filename_entry.grid(row=0, column=1, padx=10, pady=10, sticky="ew")
                
                # Directory selection
                ttk.Label(filename_dialog, text="Save location:").grid(row=1, column=0, padx=10, pady=10, sticky="w")
                directory_var = tk.StringVar(value=os.getcwd())
                directory_frame = ttk.Frame(filename_dialog)
                directory_frame.grid(row=1, column=1, padx=10, pady=10, sticky="ew")
                
                directory_entry = ttk.Entry(directory_frame, textvariable=directory_var, width=25)
                directory_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
                
                browse_btn = ttk.Button(directory_frame, text="Browse...", 
                                        command=lambda: directory_var.set(filedialog.askdirectory()))
                browse_btn.pack(side=tk.RIGHT, padx=(5, 0))
                
                # Save button
                def save_and_close():
                    result = self.save_data_to_csv(filename_var.get(), directory_var.get())
                    messagebox.showinfo("Save Result", f"Data saved to: {result}")
                    filename_dialog.destroy()
                    
                save_btn = ttk.Button(filename_dialog, text="Save", command=save_and_close)
                save_btn.grid(row=2, column=0, columnspan=2, pady=10)
                
                # Configure grid weights
                filename_dialog.columnconfigure(1, weight=1)
                
                # Wait for dialog to close
                self.root.wait_window(filename_dialog)
        
        # Send final message to microcontroller
        final_message = '0\n'
        # Try to read any pending response first
        if self.serial_connection and self.serial_connection.is_open and self.serial_connection.in_waiting > 0:
            response = self.serial_connection.readline().decode('utf-8').strip()
            print(response)
        # Send shutdown command
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(final_message.encode('utf-8'))
            self.serial_connection.close()
        
        # Close encoders
        self.encoder_one.close_encoder()
        self.encoder_two.close_encoder()
        self.encoder_three.close_encoder()
        
        self.root.destroy()
        sys.exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    app = ThreeTetherGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)  # Handle window close button
    root.mainloop()
