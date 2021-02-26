from mobilerack_pybind_module import ValveController
import time
from time import sleep
import serial
import csv
import threading

# File to control valves and simultaneously log load cell data from arduino on serial port

# Set parameters
valves = [14, 15] # 14: left chaber; 15: right chamber
max_pressure = 700
ser = serial.Serial('/dev/ttyACM0')
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
            writer.writerow([time_reading*0.001 , force_reading])
        sleep(0.1)

# Create controller object
vc = ValveController("192.168.0.100", valves, max_pressure)

# Start logging thread
print('Trying to start thread')
log_thread = threading.Thread(target=log_function, aargs=(1,ser), daemon=True)
log_thread.start()
i = 0
# Controller test
sleep(5)
vc.setSinglePressure(0, 200)
sleep(5)
vc.setSinglePressure(0, 0)
sleep(3)
vc.setSinglePressure(0, 400)
sleep(5)
vc.setSinglePressure(0, 0)
sleep(3)
vc.setSinglePressure(0, 600)
sleep(5)
vc.setSinglePressure(0, 0)
sleep(3)
vc.setSinglePressure(1, 200)
sleep(5)
vc.setSinglePressure(1, 0)
sleep(3)
vc.setSinglePressure(1, 400)
sleep(5)
vc.setSinglePressure(1, 0)
sleep(3)
vc.setSinglePressure(1, 600)
sleep(5)
vc.setSinglePressure(1, 0)
sleep(3)
vc.disconnect()
# turn off arduino LED
string = 'e'
bytestring = string.encode()
ser.write(bytestring)

