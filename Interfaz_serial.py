# Una interfaz sencilla para mosrtrar losd datos UART del psoc 5LP

import tkinter as tk
import serial
import serial.tools.list_ports
from threading import Thread, Event
import re

class SensorMonitorApp:
    def __init__(self, master):
        self.master = master
        master.title("Monitor de Signos Vitales")
        master.geometry("400x300")
        
        # Variables para datos
        self.spo2_var = tk.StringVar(value="-- %")
        self.bpm_var = tk.StringVar(value="-- lpm")
        self.connection_status = tk.StringVar(value="Desconectado")
        self.serial_port = tk.StringVar()
        
        # Estilo
        title_font = ("Arial", 16, "bold")
        data_font = ("Arial", 24, "bold")
        status_font = ("Arial", 10)
        
        # Frame principal
        main_frame = tk.Frame(master, padx=20, pady=20)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Título
        tk.Label(main_frame, text="Monitor MAX30102", font=title_font).grid(row=0, column=0, columnspan=2, pady=10)
        
        # Datos SpO2
        tk.Label(main_frame, text="SpO2:").grid(row=1, column=0, sticky='e', padx=10)
        tk.Label(main_frame, textvariable=self.spo2_var, font=data_font, fg="blue").grid(row=1, column=1, sticky='w')
        
        # Datos BPM
        tk.Label(main_frame, text="BPM:").grid(row=2, column=0, sticky='e', padx=10)
        tk.Label(main_frame, textvariable=self.bpm_var, font=data_font, fg="red").grid(row=2, column=1, sticky='w')
        
        # Selector de puerto serial
        port_frame = tk.Frame(main_frame)
        port_frame.grid(row=3, column=0, columnspan=2, pady=15)
        
        tk.Label(port_frame, text="Puerto COM:").pack(side=tk.LEFT)
        
        ports = self.get_serial_ports()
        port_dropdown = tk.OptionMenu(port_frame, self.serial_port, *ports)
        port_dropdown.pack(side=tk.LEFT, padx=5)
        self.serial_port.set(ports[0] if ports else "")
        
        # Botones de control
        self.connect_button = tk.Button(main_frame, text="Conectar", command=self.toggle_connection)
        self.connect_button.grid(row=4, column=0, pady=10)
        
        tk.Button(main_frame, text="Salir", command=self.close_app).grid(row=4, column=1, pady=10)
        
        # Estado de conexión
        tk.Label(main_frame, textvariable=self.connection_status, font=status_font, fg="gray").grid(row=5, column=0, columnspan=2)
        
        # Variables de control
        self.serial_connection = None
        self.read_thread = None
        self.running = Event()
        
    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def toggle_connection(self):
        if self.running.is_set():
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        port = self.serial_port.get()
        if not port:
            return
            
        try:
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=9600,        # Debe coincidir con la configuración del PSoC
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            
            self.running.set()
            self.read_thread = Thread(target=self.read_serial_data)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            self.connection_status.set(f"Conectado a {port}")
            self.connect_button.config(text="Desconectar")
            
        except serial.SerialException as e:
            self.connection_status.set(f"Error: {str(e)}")
    
    def disconnect_serial(self):
        self.running.clear()
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.connection_status.set("Desconectado")
        self.connect_button.config(text="Conectar")
    
    def read_serial_data(self):
        while self.running.is_set():
            try:
                if self.serial_connection.in_waiting:
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    self.parse_sensor_data(line)
            except (serial.SerialException, UnicodeDecodeError) as e:
                print(f"Error serial: {e}")
                self.disconnect_serial()
                break
    
    def parse_sensor_data(self, data):
        # Buscar patrones: "SpO2: XX%, BPM: YY"
        spo2_match = re.search(r'SpO2:\s*(\d+)%', data)
        bpm_match = re.search(r'BPM:\s*(\d+)', data)
        
        if spo2_match and bpm_match:
            spo2_value = spo2_match.group(1)
            bpm_value = bpm_match.group(1)
            
            self.spo2_var.set(f"{spo2_value} %")
            self.bpm_var.set(f"{bpm_value} lpm")
    
    def close_app(self):
        self.disconnect_serial()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SensorMonitorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close_app)
    root.mainloop()