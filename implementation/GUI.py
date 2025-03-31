import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
from Dynamic_model_class import three_teth_model
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
        self.three_teth_calc = three_teth_model()
        #self.encoder_one = phidget_encoder(0)
        
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
        
        # Initialize command vector
        self.command_vec = []
        self.current_state = 1
        
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
            
        except:
            messagebox.showerror("Input Error", "Please ensure all input fields are valid numbers.")
    
    def start_operation(self):
        self.running = True
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        
        # Start the operation in a separate thread
        self.operation_thread = threading.Thread(target=self.run_operation)
        self.operation_thread.daemon = True
        self.operation_thread.start()
    
    def stop_operation(self):
        self.running = False
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
    
    def set_state(self, state):
        if self.running:
            self.current_state = state
            self.data_text.insert(tk.END, f"Changed to State {state}\n")
            self.data_text.see(tk.END)
    
    def run_operation(self):
        j = 0
        
        while self.running:
            try:
                # Determine current operation mode
                current_mode = "Manual" if self.manual_control.get() else "Automatic"
                
                # Record timestamps
                current_utc = datetime.now(timezone.utc)
                current_microsec = int(time_module.time() * 1000000 - self.start_time_microsec)
                self.time_utc.append(current_utc)
                self.time_microsec.append(current_microsec)
                
                # Record operation mode
                self.operation_modes.append(current_mode)
                
                # Record the last command or empty string if no commands yet
                latest_command = self.command_vec[-1].strip() if self.command_vec else ""
                self.sent_commands.append(latest_command)

                if j == 0:
                    initial_guess = [0, 0, -3]
                else:
                    initial_guess = self.apex[-1]
                
               
                #self.encoder_one.get_angle()
                #tether1_length = self.encoder_one.angle2length(float(self.tether_one_length.get()))
                tether1_length = float(self.tether_one_length.get())
                tether2_length = float(self.tether_two_length.get())
                tether3_length = float(self.tether_three_length.get())
                
                self.tether_length_one.append(tether1_length)
                self.tether_length_two.append(tether2_length)
                self.tether_length_three.append(tether3_length)
                
                # Calculate apex position
                apex_position = self.three_teth_calc.calculate_apex(tether1_length/12, tether2_length/12, tether3_length/12, initial_guess)
                self.apex.append(apex_position)
                
                # Calculate forces
                forces = self.three_teth_calc.calculate_tether_forces(apex_position)
                self.tether_force_one.append(forces[0][0])
                self.tether_force_two.append(forces[1][0])
                self.tether_force_three.append(forces[2][0])
                self.states.append(self.current_state)

                # Format the output string with the current state and mode
                output_text = f"Mode: {current_mode}, State: {self.current_state}, "
                output_text += f"Lengths: {tether1_length:.4f}, {tether2_length:.4f}, {tether3_length:.4f}\n"
                output_text += f"Apex: {apex_position}\n"
                output_text += f"Forces: {forces[0][0]:.4f}, {forces[1][0]:.4f}, {forces[2][0]:.4f}\n"
                
                # Add command info if in manual mode
                if current_mode == "Manual" and latest_command:
                    output_text += f"Last Command: {latest_command}\n"
                    
                output_text += "-" * 50 + "\n"
                
                # Update the UI with the output
                self.root.after(0, lambda: self.update_data_display(output_text))
                
                time.sleep(0.1)  # Same as in the original script
                j += 1
                
            except Exception as e:
                error_msg = f"Error in operation: {str(e)}\n"
                self.root.after(0, lambda: self.update_data_display(error_msg))
                time.sleep(1)
    
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
                
                # Write header row with additional columns
                csvwriter.writerow([
                    'UTC Time', 'Microseconds Since Start', 
                    'Tether 1 Length', 'Tether 2 Length', 'Tether 3 Length',
                    'X Apex', 'Y Apex', 'Z Apex',
                    'Tether 1 Force', 'Tether 2 Force', 'Tether 3 Force',
                    'Current State', 'Operation Mode', 'Sent Command'
                ])
                
                # Write data rows
                for i in range(len(self.time_utc)):
                    # Only proceed if we have all data for this row
                    if (i < len(self.apex) and i < len(self.tether_force_one)):
                        command = self.sent_commands[i] if i < len(self.sent_commands) else ""
                        mode = self.operation_modes[i] if i < len(self.operation_modes) else "Automatic"
                        
                        csvwriter.writerow([
                            self.time_utc[i],
                            self.time_microsec[i],
                            self.tether_length_one[i],
                            self.tether_length_two[i],
                            self.tether_length_three[i],
                            self.apex[i][0],  # X coordinate
                            self.apex[i][1],  # Y coordinate
                            self.apex[i][2],  # Z coordinate
                            self.tether_force_one[i],
                            self.tether_force_two[i],
                            self.tether_force_three[i],
                            self.states[i],
                            mode,
                            command
                        ])
            
            return filepath  # Return the filepath for confirmation message
        except Exception as e:
            return f"Error saving data: {str(e)}"

    def on_closing(self):
        if self.running:
            self.stop_operation()
        
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
                dir_var = tk.StringVar(value=os.getcwd())
                dir_display = ttk.Entry(filename_dialog, textvariable=dir_var, width=30, state="readonly")
                dir_display.grid(row=1, column=1, padx=10, pady=10, sticky="ew")
                browse_btn = ttk.Button(filename_dialog, text="Browse...", 
                                     command=lambda: dir_var.set(filedialog.askdirectory()))
                browse_btn.grid(row=1, column=2, padx=5, pady=10)
                
                # Save button
                def save_with_custom_name():
                    result = self.save_data_to_csv(filename_var.get(), dir_var.get())
                    filename_dialog.destroy()
                    if result.startswith("Error"):
                        messagebox.showerror("Save Error", result)
                    else:
                        messagebox.showinfo("Save Successful", f"Data saved to {result}")
                    self.root.destroy()
                
                save_btn = ttk.Button(filename_dialog, text="Save", command=save_with_custom_name)
                save_btn.grid(row=2, column=0, columnspan=3, padx=10, pady=10)
                
                # Handle dialog close
                def on_dialog_close():
                    filename_dialog.destroy()
                    self.root.destroy()
                
                filename_dialog.protocol("WM_DELETE_WINDOW", on_dialog_close)
                
                # Wait for this dialog to be processed
                return
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        
        self.root.destroy()

    def monitor_serial_response(self, original_command):
        response = ""
        start_time = time.time()
        last_display_time = 0
        display_interval = 0.002  # 2 milliseconds
        
        # Keep checking for responses until a new command is sent or timeout occurs
        while (len(self.command_vec) > 0 and 
              self.command_vec[-1] == original_command):  # 30 second timeout
            
            try:
                current_time = time.time()
                
                # Read all available data from the buffer
                while self.serial_connection.in_waiting > 0:
                    incoming = self.serial_connection.readline().decode('utf-8').strip()
                    if incoming:
                        response = incoming
                        
                        # Only display if enough time has passed since last display
                        if current_time - last_display_time >= display_interval:
                            # Use after method to safely update UI from a background thread
                            self.root.after(0, lambda r=response: self.update_data_display(f"Response: {r}\n"))
                            last_display_time = current_time
                
                # Small delay to prevent CPU hogging
                time.sleep(0.001)  # 1ms sleep to check more frequently
                
            except Exception as e:
                self.root.after(0, lambda: self.update_data_display(f"Error reading response: {str(e)}\n"))
                break
        
        # Add a separator when finished
        self.root.after(0, lambda: self.update_data_display("-" * 50 + "\n"))

    def send_manual_command(self):
        if not self.manual_control.get():
            messagebox.showinfo("Manual Control", "Please enable manual control first")
            return
            
        command = self.command_entry.get().strip()
        if not command:
            messagebox.showinfo("Manual Control", "Please enter a command")
            return
            
        try:
            # Check if serial connection is open
            if not self.serial_connection or not self.serial_connection.is_open:
                messagebox.showerror("Connection Error", "Serial connection is not open")
                return
                
            # Add newline character if not present (often needed for Arduino/microcontroller communication)
            if not command.endswith('\n'):
                command += '\n'
                
            # Send command to microcontroller
            self.serial_connection.write(command.encode('utf-8'))
            
            # Keep track of sent commands
            self.command_vec.append(command)

            # Display in the UI that the command was sent
            output_text = f"Manual Command Sent: {command.strip()}\n"
            self.update_data_display(output_text)

            # Set up a monitoring thread to continuously check for responses
            self.monitor_thread = threading.Thread(
                target=self.monitor_serial_response,
                args=(command,)
            )
            self.monitor_thread.daemon = True
            self.monitor_thread.start()

            # Clear the command entry
            self.command_entry.delete(0, tk.END)
            
        except Exception as e:
            error_msg = f"Error in serial communication: {str(e)}\n"
            self.update_data_display(error_msg)

if __name__ == "__main__":
    root = tk.Tk()
    app = ThreeTetherGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
