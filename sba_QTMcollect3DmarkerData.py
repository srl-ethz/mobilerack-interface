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

print("Enter file name")
fileName = input()
print("nr of markers?")
number_of_markers = int(input())

rec_time = 10                   # recording time [s]
time_int = 0.01                 # time interval between steps
timesteps = rec_time/time_int   # nr of recordings



# number_of_markers = 8
headerList = ["timestamp"]
for i in range(number_of_markers):
    headerList.append(["x"+str(i),"y"+str(i),"z"+str(i)])
    
file = open('ExportData/'+fileName+'.txt', 'w')
file.write(str(headerList)+'\n')

# print(headerList)
qc = QualisysClient(number_of_markers, cameras, "3DNoLabels")

sleep(1)  # hacky way to wait until data from qtm is received
# _, timestamp = qc.getData3D()
# vc.syncTimeStamp(timestamp//1000)  # sync the time to be that of QTM

captured_markers = []


start_time = time.time()
# start_time = datetime.datetime.now()
print(start_time)

frame_list = []
# curr_stamp = timestamp

file = open('ExportData/'+fileName+'.txt', 'a')
# curr_stamp = datetime.datetime.now()
curr_stamp = time.time()
timestamp = curr_stamp

# for t in range(timesteps):
for t in IntervalTimer(time_int, stop=timesteps):
    dataList = []
    frames, _ = qc.getData3D()
    # print(frames)
    print(str(qc.getData3D()))
    timestamp = time.time()
    dataList.append(timestamp)
    for m in range(number_of_markers):
        dataList.append([])
        dataList[m+1].append(frames[m][0])
        dataList[m+1].append(frames[m][1])
        dataList[m+1].append(frames[m][2])
    file.writelines(str(dataList)+'\n')

            
total_time = time.time()-start_time
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
