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
        self.encoder_one = phidget_encoder(0)
        self.encoder_two = phidget_encoder(1)
        self.encoder_three = phidget_encoder(2)
        
        # User inputs
        self.motor_count = tk.StringVar()
        self.user_mass = tk.StringVar()
        self.user_height = tk.StringVar()
        self.user_waist = tk.StringVar()
        self.tether_one_length = tk.StringVar()
        self.tether_two_length = tk.StringVar()
        self.tether_three_length = tk.StringVar()
        
        # Data storage
        self.time_utc = []
        self.time_microsec = []
        self.start_time_microsec = time_module.time() * 1000000  # Start time in microseconds
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
        
        # Initialize command vector
        self.command_vec = []
        self.current_state = 3  # Initialize state to 3
        
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
        
        # Configuration frame
        config_frame = ttk.LabelFrame(self.root, text="System Configuration")
        config_frame.pack(fill="x", padx=10, pady=10)
        
        # Number of motors
        ttk.Label(config_frame, text="Number of Motors:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        motor_entry = ttk.Entry(config_frame, textvariable=self.motor_count)
        motor_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        # User mass
        ttk.Label(config_frame, text="User Mass (lbs):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        mass_entry = ttk.Entry(config_frame, textvariable=self.user_mass)
        mass_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        # User height
        ttk.Label(config_frame, text="User Height (inches):").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        height_entry = ttk.Entry(config_frame, textvariable=self.user_height)
        height_entry.grid(row=2, column=1, padx=5, pady=5, sticky="ew")
        
        # User waist
        ttk.Label(config_frame, text="User Waist Circumference (inches):").grid(row=3, column=0, padx=5, pady=5, sticky="w")
        waist_entry = ttk.Entry(config_frame, textvariable=self.user_waist)
        waist_entry.grid(row=3, column=1, padx=5, pady=5, sticky="ew")
        
        # Tether lengths
        ttk.Label(config_frame, text="Tether 1 Length (inches):").grid(row=4, column=0, padx=5, pady=5, sticky="w")
        tether1_entry = ttk.Entry(config_frame, textvariable=self.tether_one_length)
        tether1_entry.grid(row=4, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(config_frame, text="Tether 2 Length (inches):").grid(row=5, column=0, padx=5, pady=5, sticky="w")
        tether2_entry = ttk.Entry(config_frame, textvariable=self.tether_two_length)
        tether2_entry.grid(row=5, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(config_frame, text="Tether 3 Length (inches):").grid(row=6, column=0, padx=5, pady=5, sticky="w")
        tether3_entry = ttk.Entry(config_frame, textvariable=self.tether_three_length)
        tether3_entry.grid(row=6, column=1, padx=5, pady=5, sticky="ew")
        
        # Confirm button
        confirm_btn = ttk.Button(config_frame, text="Confirm", command=self.confirm_configuration)
        confirm_btn.grid(row=7, column=0, columnspan=2, padx=5, pady=10)
        
        # Configure grid weights
        config_frame.columnconfigure(1, weight=1)
    
    def create_operation_ui(self):
        # Clear previous UI
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # Operation frame
        operation_frame = ttk.LabelFrame(self.root, text="System Operation")
        operation_frame.pack(fill="x", padx=10, pady=10)
        
        # Start/Stop buttons
        self.start_btn = ttk.Button(operation_frame, text="Start", command=self.start_operation)
        self.start_btn.grid(row=0, column=0, padx=5, pady=10)
        
        self.stop_btn = ttk.Button(operation_frame, text="Stop", command=self.stop_operation, state="disabled")
        self.stop_btn.grid(row=0, column=1, padx=5, pady=10)
        
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
        # Validate inputs
        try:
            # Basic validation - just make sure these can be converted to numbers
            motor_count = int(self.motor_count.get())
            float(self.user_mass.get())
            float(self.user_height.get())
            float(self.user_waist.get())
            float(self.tether_one_length.get())
            float(self.tether_two_length.get())
            float(self.tether_three_length.get())
            
            # Switch to operation UI
            self.create_operation_ui()
            
        except ValueError:
            messagebox.showerror("Input Error", "Please ensure all input fields are valid numbers.")
    
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
    
    def get_tether_lengths_and_calculate_apex(self, initial_guess=None):
        if initial_guess is None and len(self.apex) > 0:
            initial_guess = self.apex[-1]
        elif initial_guess is None:
            initial_guess = [0, 0, -3]
        
        # Get tether lengths from encoders
        tether1_length = float(self.encoder_one.angle2length(self.tether_one_length.get()))
        tether2_length = float(self.encoder_two.angle2length(self.tether_two_length.get()))
        tether3_length = float(self.encoder_three.angle2length(self.tether_three_length.get()))
        
        # Calculate apex position
        apex_position = self.three_teth_calc.calculate_apex(
            tether1_length/12, tether2_length/12, tether3_length/12, initial_guess
        )
        
        # Calculate forces
        forces = self.three_teth_calc.calculate_tether_forces(apex_position)
        
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
                # Record timestamps
                current_utc = datetime.now(timezone.utc)
                current_microsec = int(time_module.time() * 1000000 - self.start_time_microsec)
                
                # Calculate tether lengths and apex
                initial_guess = self.apex[-1] if j > 0 else [0, 0, -3]
                tether1_length, tether2_length, tether3_length, apex_position, forces = \
                    self.get_tether_lengths_and_calculate_apex(initial_guess)
                
                # Record data
                self.time_utc.append(current_utc)
                self.time_microsec.append(current_microsec)
                self.operation_modes.append("Automatic")
                self.tether_length_one.append(tether1_length)
                self.tether_length_two.append(tether2_length)
                self.tether_length_three.append(tether3_length)
                self.apex.append(apex_position)
                self.tether_force_one.append(forces[0][0])
                self.tether_force_two.append(forces[1][0])
                self.tether_force_three.append(forces[2][0])
                self.states.append(self.current_state)
                
                # Format command with all required parameters
                command = f'{self.current_state},{self.tether_force_one[-1]:.4f},{self.tether_force_two[-1]:.4f},{self.tether_force_three[-1]:.4f}\n'
                
                # Send command to microcontroller
                self.serial_connection.write(command.encode('utf-8'))
                command_time = time_module.time()
                
                # Keep track of sent commands
                self.command_vec.append(command)
                self.sent_commands.append(command.strip())
                
                # Format the output string with the current state and mode
                output_text = f"Mode: Automatic, State: {self.current_state}\n"
                output_text += f"Lengths: {tether1_length:.4f}, {tether2_length:.4f}, {tether3_length:.4f}\n"
                output_text += f"Apex: {apex_position}\n"
                output_text += f"Forces: {forces[0][0]:.4f}, {forces[1][0]:.4f}, {forces[2][0]:.4f}\n"
                output_text += f"Command: {command.strip()}\n"
                
                # Update the UI with the command output
                self.root.after(0, lambda t=output_text: self.update_data_display(t))
                
                # Wait for response (non-blocking, will immediately process any response)
                response = None
                while response is None:
                    if not self.running or self.manual_control.get():
                        # Exit loop if stopped or switched to manual mode
                        return
                        
                    if self.serial_connection.in_waiting > 0:
                        response = self.serial_connection.readline().decode('utf-8').strip()
                        if response:
                            self.process_response(response, j)
                            break
                    
                    # Small sleep to prevent CPU hogging
                    time.sleep(0.001)
                
                # Increment counter
                j += 1
                
                # No delay between iterations - run as fast as possible
            
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
    
    def process_response(self, response, index):
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
                
                # Display response
                response_text = f"Response: {response}\n"
                response_text += f"Loadcell forces: {parts[1]}, {parts[2]}, {parts[3]}\n"
                response_text += "-" * 50 + "\n"
                self.root.after(0, lambda t=response_text: self.update_data_display(t))
            else:
                # Invalid response format
                self.root.after(0, lambda r=response: self.update_data_display(f"Response: {r}\n"))
                self.response_times.append(None)
                self.loadcell_force_one.append(None)
                self.loadcell_force_two.append(None)
                self.loadcell_force_three.append(None)
        
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
                csvwriter.writerow([
                    'UTC Time', 'Microseconds Since Start', 
                    'Tether 1 Length', 'Tether 2 Length', 'Tether 3 Length',
                    'X Apex', 'Y Apex', 'Z Apex',
                    'Tether 1 Force', 'Tether 2 Force', 'Tether 3 Force',
                    'Current State', 'Operation Mode', 'Sent Command',
                    'Response Time', 'Loadcell 1 Force', 'Loadcell 2 Force', 'Loadcell 3 Force', 'Raw Response'
                ])
                
                # Write data rows
                for i in range(len(self.time_utc)):
                    # Get values, using None for missing data
                    command = self.sent_commands[i] if i < len(self.sent_commands) else ""
                    mode = self.operation_modes[i] if i < len(self.operation_modes) else "Automatic"
                    apex_x = self.apex[i][0] if i < len(self.apex) else None
                    apex_y = self.apex[i][1] if i < len(self.apex) else None
                    apex_z = self.apex[i][2] if i < len(self.apex) else None
                    response = self.responses[i] if i < len(self.responses) else ""
                    resp_time = self.response_times[i] if i < len(self.response_times) else None
                    lc1 = self.loadcell_force_one[i] if i < len(self.loadcell_force_one) else None
                    lc2 = self.loadcell_force_two[i] if i < len(self.loadcell_force_two) else None
                    lc3 = self.loadcell_force_three[i] if i < len(self.loadcell_force_three) else None
                    
                    csvwriter.writerow([
                        self.time_utc[i],
                        self.time_microsec[i],
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
                        response
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
                manual_state = int(command_parts[0])
                manual_force1 = float(command_parts[1])
                manual_force2 = float(command_parts[2])
                manual_force3 = float(command_parts[3])
            else:
                self.update_data_display("Error: Command must be in format: state,force1,force2,force3\n")
                return
                
            # Check if serial connection is open
            if not self.serial_connection or not self.serial_connection.is_open:
                messagebox.showerror("Connection Error", "Serial connection is not open")
                return
                
            # Add newline character if not present
            if not command.endswith('\n'):
                command += '\n'
                
            # Record timestamps
            current_utc = datetime.now(timezone.utc)
            current_microsec = int(time_module.time() * 1000000 - self.start_time_microsec)
            
            # Get tether lengths and calculate apex positions (same as automatic mode)
            tether1_length, tether2_length, tether3_length, apex_position, _ = \
                self.get_tether_lengths_and_calculate_apex()
            
            # Store data for this manual command
            self.time_utc.append(current_utc)
            self.time_microsec.append(current_microsec)
            self.operation_modes.append("Manual")
            self.tether_length_one.append(tether1_length)
            self.tether_length_two.append(tether2_length)
            self.tether_length_three.append(tether3_length)
            self.apex.append(apex_position)
            self.states.append(manual_state)
            self.tether_force_one.append(manual_force1)
            self.tether_force_two.append(manual_force2)
            self.tether_force_three.append(manual_force3)
            self.sent_commands.append(command.strip())
                
            # Send command to microcontroller
            self.serial_connection.write(command.encode('utf-8'))
            
            # Keep track of sent commands
            self.command_vec.append(command)

            # Display in the UI that the command was sent
            output_text = f"Manual Command Sent: {command.strip()}\n"
            output_text += f"Lengths: {tether1_length:.4f}, {tether2_length:.4f}, {tether3_length:.4f}\n"
            output_text += f"Apex: {apex_position}\n"
            self.update_data_display(output_text)

            # Start a background thread to continuously monitor for responses
            # but only if we don't already have one running
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
        response_index = len(self.time_utc) - 1
        
        while self.manual_control.get() and self.manual_monitoring:
            try:
                if self.serial_connection.in_waiting > 0:
                    response = self.serial_connection.readline().decode('utf-8').strip()
                    if response:
                        # Process the response
                        # For continuous monitoring, we need to increment the index
                        response_index += 1
                        
                        # Make sure we have data arrays long enough
                        while len(self.time_utc) <= response_index:
                            # Duplicate the last entry for timestamps and other data
                            # with appropriate modifications
                            current_utc = datetime.now(timezone.utc)
                            current_microsec = int(time_module.time() * 1000000 - self.start_time_microsec)
                            
                            # Add placeholders for the new data point, duplicating most recent values
                            self.time_utc.append(current_utc)
                            self.time_microsec.append(current_microsec)
                            self.operation_modes.append("Manual")
                            self.tether_length_one.append(self.tether_length_one[-1])
                            self.tether_length_two.append(self.tether_length_two[-1])
                            self.tether_length_three.append(self.tether_length_three[-1])
                            self.apex.append(self.apex[-1])
                            self.states.append(self.states[-1])
                            self.tether_force_one.append(self.tether_force_one[-1])
                            self.tether_force_two.append(self.tether_force_two[-1])
                            self.tether_force_three.append(self.tether_force_three[-1])
                            self.sent_commands.append("")  # No new command was sent
                        
                        # Process and display the response
                        self.process_response(response, response_index)
                
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
        
        # Close serial connection if open
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        
        # Close encoders
        self.encoder_one.close()
        self.encoder_two.close()
        self.encoder_three.close()
        
        self.root.destroy()
        sys.exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    app = ThreeTetherGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)  # Handle window close button
    root.mainloop()
