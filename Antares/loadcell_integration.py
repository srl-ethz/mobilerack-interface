from mobilerack_pybind_module import ValveController
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
ser = serial.Serial('/dev/ttyACM0')

# Seperate thread function for logging
def log_function(name, ser):
    print('Start serial logging')
    # Open serial port and flush
    ser = serial.Serial('/dev/ttyACM0')
    ser.write('s'.encode('ASCII'))
    ser.flushInput()
    # Send sync message
    string = 's'
    bytestring = string.encode()
    ser.write(bytestring)
    starttime = time.time()
    while True:
        ser.write('s')
        # Read force from Arduino
        ser_bytes = ser.readline()
        force_reading = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        #Write data to file
        with open("force.csv","a") as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([time.time()-starttime,force_reading])
        sleep(0.1)

# Create controller object
vc = ValveController("192.168.0.100", valves, max_pressure)

# Start logging thread
log_thread = threading.Thread(target=log_function, args=(1,ser), daemon=True)
log_thread.start()
i = 0
# Controller test
while i < cycle_count:
    vc.setSinglePressure(0, pressure)
    vc.setSinglePressure(1, 0)
    sleep(cycle_time)
    vc.setSinglePressure(0, 0)
    vc.setSinglePressure(1, pressure)
    sleep(cycle_time)
    i += 1
vc.disconnect()
# turn off arduino LED
string = 'e'
bytestring = string.encode()
ser.write(bytestring)
