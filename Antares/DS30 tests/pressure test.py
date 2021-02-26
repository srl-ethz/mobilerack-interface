from mobilerack_pybind_module import ValveController
import time
from time import sleep
import serial
import csv
import threading

# File to control valves and simultaneously log load cell data from arduino on serial port

# Set parameters
valves = [14, 15] # 14: left chaber; 15: right chamber
max_pressure = 1000
ser = serial.Serial('/dev/ttyACM0')
# Seperate thread function for logging
def log_function(name):
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
        force_reading = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        #Write data to file
        with open("force.csv","a") as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([time.time()-starttime,force_reading])
        sleep(0.1)

# Create controller object
vc = ValveController("192.168.0.100", valves, max_pressure)

# Start logging thread
print('Trying to start thread')
log_thread = threading.Thread(target=log_function, args=(1,), daemon=True)
log_thread.start()
i = 0
# Controller test
vc.setSinglePressure(0, 700)
sleep(5)
vc.setSinglePressure(0, 0)
sleep(3)
vc.setSinglePressure(0, 800)
sleep(5)
vc.setSinglePressure(0, 0)
sleep(3)
vc.setSinglePressure(0, 900)
sleep(5)
vc.setSinglePressure(0, 0)
sleep(3)
vc.setSinglePressure(1, 700)
sleep(5)
vc.setSinglePressure(1, 0)
sleep(3)
vc.setSinglePressure(1, 800)
sleep(5)
vc.setSinglePressure(1, 0)
sleep(3)
vc.setSinglePressure(1, 900)
sleep(5)
vc.setSinglePressure(1, 0)
sleep(3)
vc.disconnect()
# turn off arduino LED
string = 'e'
bytestring = string.encode()
ser.write(bytestring)

