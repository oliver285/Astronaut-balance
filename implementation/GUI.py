import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import csv
import random  
from datetime import datetime

class TetherMeasurementGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Tether Measurement System")
        self.root.geometry("1200x700")
        
        
        self.tether1_var = tk.StringVar()
        self.tether2_var = tk.StringVar()
        self.tether3_var = tk.StringVar()
        self.weight_var = tk.StringVar()
        self.height_var = tk.StringVar()
        self.waist_var = tk.StringVar()
        self.csv_path = None
        self.running = False
        
        self.create_input_frame()
        self.create_graph_frame()
        
    def create_input_frame(self):
        input_frame = ttk.LabelFrame(self.root, text="Input Parameters", padding="10")
        input_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        
        
        left_frame = ttk.Frame(input_frame)
        left_frame.grid(row=0, column=0, padx=5)
        
        right_frame = ttk.Frame(input_frame)
        right_frame.grid(row=0, column=1, padx=5)
        
        
        ttk.Label(left_frame, text="Tether 1 Length:").grid(row=0, column=0, padx=5, pady=5)
        ttk.Entry(left_frame, textvariable=self.tether1_var).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(left_frame, text="inches").grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Label(left_frame, text="Tether 2 Length:").grid(row=1, column=0, padx=5, pady=5)
        ttk.Entry(left_frame, textvariable=self.tether2_var).grid(row=1, column=1, padx=5, pady=5)
        ttk.Label(left_frame, text="inches").grid(row=1, column=2, padx=5, pady=5)
        
        ttk.Label(left_frame, text="Tether 3 Length:").grid(row=2, column=0, padx=5, pady=5)
        ttk.Entry(left_frame, textvariable=self.tether3_var).grid(row=2, column=1, padx=5, pady=5)
        ttk.Label(left_frame, text="inches").grid(row=2, column=2, padx=5, pady=5)
        
        
        ttk.Label(right_frame, text="User Weight:").grid(row=0, column=0, padx=5, pady=5)
        ttk.Entry(right_frame, textvariable=self.weight_var).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(right_frame, text="lbs").grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Label(right_frame, text="User Height:").grid(row=1, column=0, padx=5, pady=5)
        ttk.Entry(right_frame, textvariable=self.height_var).grid(row=1, column=1, padx=5, pady=5)
        ttk.Label(right_frame, text="inches").grid(row=1, column=2, padx=5, pady=5)
        
        ttk.Label(right_frame, text="User Waist:").grid(row=2, column=0, padx=5, pady=5)
        ttk.Entry(right_frame, textvariable=self.waist_var).grid(row=2, column=1, padx=5, pady=5)
        ttk.Label(right_frame, text="inches").grid(row=2, column=2, padx=5, pady=5)
        
        
        button_frame = ttk.Frame(input_frame)
        button_frame.grid(row=1, column=0, columnspan=2, pady=10)
        
        
        ttk.Button(button_frame, text="Select CSV File", command=self.select_csv).pack(side=tk.LEFT, padx=5)
        
        # Start/Stop buttons
        self.start_button = ttk.Button(button_frame, text="Start", command=self.start_measurement, state="disabled")
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_measurement, state="disabled")
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
    def create_graph_frame(self):
        self.graph_frame = ttk.LabelFrame(self.root, text="Measurements", padding="10")
        self.graph_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        
        
        self.fig = plt.Figure(figsize=(12, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_frame)
        
        
        gs = self.fig.add_gridspec(2, 6)  
        
        
        self.tether1_plot = self.fig.add_subplot(gs[0, 0:2])
        self.tether2_plot = self.fig.add_subplot(gs[0, 2:4])
        self.tether3_plot = self.fig.add_subplot(gs[0, 4:6])
        
        
        self.force_error_plot = self.fig.add_subplot(gs[1, 0:3])
        self.angular_error_plot = self.fig.add_subplot(gs[1, 3:6])
        
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
       
        self.time_data = []
        self.tether1_data = []
        self.tether2_data = []
        self.tether3_data = []
        self.force_error_data = []
        self.angular_error_data = []
    
    def select_csv(self):
        self.csv_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=f"tether_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        if self.csv_path:
            self.start_button.config(state="normal")
            
    def start_measurement(self):
        
        required_inputs = {
            'Tether 1': self.tether1_var.get(),
            'Tether 2': self.tether2_var.get(),
            'Tether 3': self.tether3_var.get(),
            'Weight': self.weight_var.get(),
            'Height': self.height_var.get(),
            'Waist': self.waist_var.get()
        }
        
        missing_inputs = [key for key, value in required_inputs.items() if not value]
        if missing_inputs:
            messagebox.showerror("Error", f"Please enter all required inputs:\n{', '.join(missing_inputs)}")
            return
            
        self.running = True
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        
       
        with open(self.csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'Tether1_Length', 'Tether2_Length', 'Tether3_Length', 
                           'Force_Error', 'Angular_Error'])
            
            writer.writerow(['User Data:', '', '', '', '', ''])
            writer.writerow(['Weight (lbs)', self.weight_var.get(), '', '', '', ''])
            writer.writerow(['Height (inches)', self.height_var.get(), '', '', '', ''])
            writer.writerow(['Waist (inches)', self.waist_var.get(), '', '', '', ''])
            writer.writerow(['', '', '', '', '', '']) 
            writer.writerow(['Time', 'Tether1_Length', 'Tether2_Length', 'Tether3_Length', 
                           'Force_Error', 'Angular_Error'])
        
        self.update_plots()
        
    def stop_measurement(self):
        self.running = False
        self.stop_button.config(state="disabled")
        self.start_button.config(state="normal")
        messagebox.showinfo("Complete", f"Test completed! Data saved to:\n{self.csv_path}")
        
    def update_plots(self):
        if not self.running:
            return
            
       
        self.time_data.append(len(self.time_data))
        self.tether1_data.append(float(self.tether1_var.get()) + random.uniform(-0.5, 0.5))
        self.tether2_data.append(float(self.tether2_var.get()) + random.uniform(-0.5, 0.5))
        self.tether3_data.append(float(self.tether3_var.get()) + random.uniform(-0.5, 0.5))
        self.force_error_data.append(random.uniform(-2, 2))
        self.angular_error_data.append(random.uniform(-5, 5))
        
       
        self.tether1_plot.clear()
        self.tether2_plot.clear()
        self.tether3_plot.clear()
        self.force_error_plot.clear()
        self.angular_error_plot.clear()
        
        self.tether1_plot.plot(self.time_data, self.tether1_data)
        self.tether1_plot.set_title("Tether 1 Length")
        self.tether1_plot.set_ylabel("inches")
        
        self.tether2_plot.plot(self.time_data, self.tether2_data)
        self.tether2_plot.set_title("Tether 2 Length")
        self.tether2_plot.set_ylabel("inches")
        
        self.tether3_plot.plot(self.time_data, self.tether3_data)
        self.tether3_plot.set_title("Tether 3 Length")
        self.tether3_plot.set_ylabel("inches")
        
        self.force_error_plot.plot(self.time_data, self.force_error_data)
        self.force_error_plot.set_title("Applied Force Error")
        self.force_error_plot.set_ylabel("lbf")
        
        self.angular_error_plot.plot(self.time_data, self.angular_error_data)
        self.angular_error_plot.set_title("Applied Force Angular Error")
        self.angular_error_plot.set_ylabel("deg")
        
        self.fig.tight_layout()
        self.canvas.draw()
        
        
        with open(self.csv_path, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.time_data[-1], self.tether1_data[-1], self.tether2_data[-1],
                           self.tether3_data[-1], self.force_error_data[-1], self.angular_error_data[-1]])
        
        
        self.root.after(100, self.update_plots)

if __name__ == "__main__":
    root = tk.Tk()
    app = TetherMeasurementGUI(root)
    root.mainloop()
