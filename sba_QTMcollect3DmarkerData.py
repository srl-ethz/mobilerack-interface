from mobilerack_pybind_module import ValveController, QualisysClient
from time import sleep
import time
from datetime import datetime
from interval_timer import IntervalTimer
import numpy as np
import pandas as pd
import csv
import json
import os


cameras = [0,1,2,3,4,5,6,7,8]
# valves = [0,1

print("Enter file name")
fileName = input()
# print("nr of markers?")
# number_of_markers = int(input())

number_of_markers = 8
rec_time = 20                   # recording time [s]
time_int = 0.01                 # time interval between steps
timesteps = rec_time/time_int   # nr of recordings

# number_of_markers = 8
# headerList = [["timestamp"]]
# for i in range(number_of_markers):
#     headerList.append(["x"+str(i+1),"y"+str(i+1),"z"+str(i+1)])
    
# file = open('ExportData/'+fileName+'.txt', 'w')
# file.write(str(headerList)+'\n')

qc = QualisysClient(number_of_markers, cameras, "3DNoLabels")
if not os.path.exists('./ExportData/'+fileName):
    os.mkdir('./ExportData/'+fileName)

sleep(1)  # hacky way to wait until data from qtm is received

captured_markers = []


start_time = time.time()
print(start_time)

frame_list = []

# file = open('ExportData/'+fileName+'.txt', 'a')
curr_stamp = time.time()
timestamp = curr_stamp

dataDict = {
    "timestamp": [],
    "points": []
}

i = 1
now = datetime.now()
for t in IntervalTimer(time_int, stop=timesteps):
    # dataList = [[]]
    points = []
    frames, _ = qc.getData3D()
    print(str(qc.getData3D()))
    timestamp = time.time()
    # dataList[0].append(timestamp-start_time)
    dataDict["timestamp"].append(timestamp-start_time)
    for m in range(number_of_markers):
        # dataList.append([])
        # dataList[m+1].append(frames[m][0])
        # dataList[m+1].append(frames[m][1])
        # dataList[m+1].append(frames[m][2])
        points.append([])
        points[m].append(frames[m][0])
        points[m].append(frames[m][1])
        points[m].append(frames[m][2])
    # print(points)
    # print()
    # file.writelines(str(dataList)+'\n')
    dataDict["points"].append(points)
    if i % 1000 == 0:
        with open("ExportData/"+fileName+"/"+fileName+"_"+now.strftime("%Y-%m-%d_%H-%M-%S")+".json", "w") as file:
            json.dump(dataDict, file)
            now = datetime.now()
            dataDict = {"timestamp": [], "points": []}
    print(i)
    i += 1

# print(dataDict)
# with open("ExportData/"+fileName+".json", "w") as file:
#     json.dump(dataDict, file)
            
total_time = time.time()-start_time
captured_markers.append(np.array(frame_list))     # frame_list has shape [T, N, 3] for N motion markers




sleep(1)