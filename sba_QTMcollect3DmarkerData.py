from mobilerack_pybind_module import ValveController, QualisysClient
from time import sleep
import time
import datetime
from interval_timer import IntervalTimer
import numpy as np
import pandas as pd
import csv


cameras = [0,1,2,3,4,5,6,7]
valves = [0,1,2]
max_pressure = 400

rec_time = 10                   # recording time [s]
time_int = 0.01                 # time interval between steps
timesteps = rec_time/time_int   # nr of recordings

number_of_markers = 8
headerList = ["timestamp"]
for i in range(number_of_markers):
    headerList.append(["x"+str(i),"y"+str(i),"z"+str(i)])
    
file = open('ExportData/captured_data.txt', 'w')
file.write(str(headerList)+'\n')

# print(headerList)
qc = QualisysClient(number_of_markers, cameras, "3D")

sleep(1)  # hacky way to wait until data from qtm is received
# _, timestamp = qc.getData3D()
# vc.syncTimeStamp(timestamp//1000)  # sync the time to be that of QTM

captured_markers = []

# print("print before loop")
# for k in range(len(pressures)):
# for k in range(1):
start_time = time.time()
# start_time = datetime.datetime.now()
print(start_time)

frame_list = []
# curr_stamp = timestamp

file = open('ExportData/captured_data', 'a')
# curr_stamp = datetime.datetime.now()
curr_stamp = time.time()
timestamp = curr_stamp

# for t in range(timesteps):
for t in IntervalTimer(time_int, stop=timesteps):
    dataList = []
    # while timestamp == curr_stamp:
    # while time.time()-timestamp < time_int:
    # frames, timestamp = qc.getData3D()
    frames, _ = qc.getData3D()
    # timestamp = datetime.datetime.now()
    timestamp = time.time()
    # dataList.append(str(timestamp))
    dataList.append(timestamp)
    # print(timestamp)
    for m in range(number_of_markers):
        dataList.append([])
        # print(frames[m][0])
        # print(m)
        dataList[m+1].append(frames[m][0])
        dataList[m+1].append(frames[m][1])
        dataList[m+1].append(frames[m][2])
        
    frame_list += [np.array(frames)]
    curr_stamp = timestamp
    # print(frame_list[t])
    # print(frame_list)
    # file.writelines(str(frame_list)+'\n')
    file.writelines(str(dataList)+'\n')

            
total_time = time.time()-start_time
# total_time = datetime.datetime.now()-start_time
# print(f"Ran for {total_time:.3f}")
# print(f"Ran for {total_time}")
# print(f"Timing accuracy: {100*(total_time-timesteps/100)/timesteps/100}")

captured_markers.append(np.array(frame_list))     # frame_list has shape [T, N, 3] for N motion markers


# for i in range(len(valves)):
#     vc.setSinglePressure(i, 0)
# # Reset
sleep(1)

# captured_markers = np.array(captured_markers)
# print(captured_markers)
# captured_info = {""" 'p': np.array(pressures),  """'data': captured_markers}
# print(captured_info)
# # np.savetxt("ExportData/captured_data.csv", captured_info)
# np.savetxt("ExportData/captured_data.npy", captured_markers)
