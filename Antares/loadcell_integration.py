#from mobilerack_pybind_module import ValveController
import time
from time import sleep
import serial
import csv
import threading

# File to control valves and simultaneously log load cell data from arduino on serial port

# Set parameters
valves = [15, 14] # 14: left chaber; 15: right chamber
pressure = 200
max_pressure = 300
freq = 4
total_time = 2
cycle_time = 1/freq
cycle_count = total_time/cycle_time
ser = serial.Serial('COM3')

# Seperate thread function for logging
def log_function(name, ser):
    print('Start serial logging')
    # serial port flush
    ser.flushInput()
    # Send sync message
    string = 's'
    bytestring = string.encode()
    ser.write(bytestring)
    starttime = time.time()
    while True:
        # Read force from Arduino
        ser_bytes = ser.readline()
        force_reading = float(ser_bytes[0:5].decode("utf-8"))
        time_reading = float(ser_bytes[6:len(ser_bytes)-2].decode("utf-8"))
        #Write data to file
        with open("force.csv","a") as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([time.time()-starttime, time_reading , force_reading , len(ser_bytes)])
        sleep(0.1)

# Start logging thread
log_thread = threading.Thread(target=log_function, args=(1,ser), daemon=True)
log_thread.start()
# Create controller object
#vc = ValveController("192.168.0.100", valves, max_pressure)
#sleep(5)
#vc.setSinglePressure(0, pressure)
#vc.setSinglePressure(1, 0)
#sleep(5)
#vc.setSinglePressure(0, 0)
#vc.setSinglePressure(1, 0)
#sleep(15)
#vc.disconnect()
sleep(15)
# turn off arduino LED
string = 'e'
bytestring = string.encode()
ser.write(bytestring)
