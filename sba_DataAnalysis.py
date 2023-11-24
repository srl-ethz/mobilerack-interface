# from mobilerack_pybind_module import ValveController, QualisysClient
from time import sleep
import time
import datetime
from interval_timer import IntervalTimer
import numpy as np
import pandas as pd
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import math
# from mpl_toolkits import mplot3d


path = str(Path(__file__).parent)

print("Which file should be analized?")
# fileName=input()
fileName="test4"




### Inspired by ChatGPT ###

# import file
with open('ExportData/'+fileName+'.txt') as file:
    header = file.readlines()[0]
with open('ExportData/'+fileName+'.txt') as file:
    dataStr = file.readlines()[1:]

# convert imported file to float lists
data = []
i = 0
for line in dataStr:                                                            # Remove line break, split list in data points
    dataLine = line.strip()[1:-1].split('], [')
    i += 1
    dataTimestep = []
    for entry in dataLine:                                                      # Convert entries of single lines to float
        values = entry.replace('[', '').replace(']', '').split(', ')
        values = [float(value) for value in values if value.strip() != '']
        dataTimestep.append(values)
    data.append(dataTimestep)                                                   # Combine data lines to list
    
    
# print(data[0])
# print(data[0][0])
# print(data[0][1])
# print(data[0][1][0])


def absCoord(list3D):
    return math.sqrt(list3D[0]**2 + list3D[1]**2 + list3D[2]**2)

def absDist(list1, list2):
    return math.sqrt((list1[0]-list2[0])**2 + (list1[1]-list2[1])**2 + (list1[2]-list2[2])**2)

### Sort first data points ###
dist0 = []
j = 0
for entry in data[0]:                           # Calculate absolute value of time 0 coordinates
    if j >= 1:
        dist0.append(absCoord(entry))
    j += 1

dist0_sort = []                                 # sort and index absolute values
for index in range(len(dist0)):
    dist0_sort.append([dist0[index],index])
dist0_sort.sort()
sort_index = []
for x in dist0_sort:
    sort_index.append(x[1]+1)
        
dataSort = [[]]                                 # write list with sorted values
j = 0
for entry in data[0]:
    if j == 0:
        dataSort[0].append(data[0][0])
    else:
        dataSort[0].append(data[0][sort_index[j-1]])
    j += 1

print(data[181])
print('\n')
print(data[182])

### Sort rest of data Points
i = 1
for line in data:
    dataSort.append([])
    # Enter timestamp to dataSort
    dataSort[i].append(data[i][0])
    IdxUsed = []
    dist = []
    j = 0       # P0
    for entry in data[i-1]:                         # Calculate difference between current point and P0(t-1)
        if j >= 1:
            dist.append(absDist(data[i][j], dataSort[i-1][1]))
        j += 1
        
    dist_sort = []                                  # sort and index differences
    for index in range(len(dist)):
        dist_sort.append([dist[index],index])
    dist_sort.sort()
    sort_index = []
    for x in dist_sort:
        sort_index.append(x[1]+1)
    
    dataSort[i].append(data[i][sort_index[0]])
    IdxUsed.append(sort_index[0])
    
    # Search for P1, enter to dataSort
    # j = 0       # P1
    # for entry in data[i]:
    #     if (j >= 1) and (not j in IdxUsed) and (absDist(data[i][j], dataSort[i-1][2]) < 0.004):
    #         dataSort[i].append(data[i][j])
    #         IdxUsed.append(j)
    #         break
    #     j += 1
   
    dist = []
    j = 0
    for entry in data[i-1]:                         # Calculate difference between current point and P1(t-1)
        if j >= 1:
            dist.append(absDist(data[i][j], dataSort[i-1][2]))
        j += 1
        
    dist_sort = []                                  # sort and index differences
    for index in range(len(dist)):
        dist_sort.append([dist[index],index])
    dist_sort.sort()
    sort_index = []
    for x in dist_sort:
        sort_index.append(x[1]+1)
    
    dataSort[i].append(data[i][sort_index[0]])
    IdxUsed.append(sort_index[0])
    
    # Search for P2, enter to dataSort
    j = 0
    for entry in data[i]:
        if (j >= 1) and (not j in IdxUsed) and (0.018 < absDist(data[i][j], dataSort[i][2]) < 0.038):
            dataSort[i].append(data[i][j])
            IdxUsed.append(j)
            break
        j += 1
    # Search for P3, enter to dataSort
    j = 0
    for entry in data[i]:
        if (j >= 1) and (not j in IdxUsed) and (0.026 < absDist(data[i][j], dataSort[i][3]) < 0.041):
            dataSort[i].append(data[i][j])
            IdxUsed.append(j)
            break
        j += 1
    # Search for P4, enter to dataSort
    j = 0
    for entry in data[i]:
        if (j >= 1) and (not j in IdxUsed) and (0.005 < absDist(data[i][j], dataSort[i][4]) < 0.020):
            dataSort[i].append(data[i][j])
            IdxUsed.append(j)
            break
        j += 1
    # Search for P5, enter to dataSort
    j = 0
    for entry in data[i]:
        if (j >= 1) and (not j in IdxUsed) and (0.014 < absDist(data[i][j], dataSort[i][5]) < 0.029):
            dataSort[i].append(data[i][j])
            IdxUsed.append(j)
            break
        j += 1
    # Search for P6, enter to dataSort
    j = 0
    for entry in data[i]:
        if (j >= 1) and (not j in IdxUsed) and (0.005 < absDist(data[i][j], dataSort[i][6]) < 0.017):
            dataSort[i].append(data[i][j])
            IdxUsed.append(j)
            break
        j += 1
    # Search for P7, enter to dataSort
    j = 0
    for entry in data[i]:
        if (j >= 1) and (not j in IdxUsed) and (0.008 < absDist(data[i][j], dataSort[i][7]) < 0.024):
            dataSort[i].append(data[i][j])
            IdxUsed.append(j)
            break
        j += 1
    
    i += 1
            



# Rearrange data to coordinate lists / Plot Trajectory
x = []; y = []; z=[]
j = 0
while j < len(dataTimestep)-1:
    x.append([])
    y.append([])
    z.append([])
    i = 0
    # for line in data:
    #     x[j].append(data[i][j+1][0])
    #     y[j].append(data[i][j+1][1])
    #     z[j].append(data[i][j+1][2])
    #     i += 1
    for line in dataSort:
        x[j].append(dataSort[i][j+1][0])
        y[j].append(dataSort[i][j+1][1])
        z[j].append(dataSort[i][j+1][2])
        i += 1
    plt.plot(y[j], z[j], 'x')
    # plt.plot(y[j], z[j])
    j += 1
plt.xlim([0,0.3])
plt.ylim([-0.1,0.1])
plt.show()