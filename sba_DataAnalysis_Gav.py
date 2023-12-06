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
from sklearn.metrics import pairwise_distances
# from mpl_toolkits import mplot3d
from tqdm import trange, tqdm

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
    # return math.sqrt((list1[0]-list2[0])**2 + (list1[1]-list2[1])**2 + (list1[2]-list2[2])**2)
    return math.sqrt((list1[1]-list2[1])**2 + (list1[2]-list2[2])**2)

### Sort first data points at t=0 ###
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

# print(data[181])
# print('\n')
# print(data[182])

marker_pos3d_arr = data

prev_tstamp, prev_pos = marker_pos3d_arr[0][0], marker_pos3d_arr[0][1:]
prev_pos = np.array(prev_pos)

for i in trange(1, len(marker_pos3d_arr)):

    curr_tstamp, curr_pos = marker_pos3d_arr[i][0], marker_pos3d_arr[i][1:]
    curr_pos = np.array(curr_pos)

    pdist = pairwise_distances(prev_pos, curr_pos)
    pdist_argmin = np.argmin(pdist, axis=1)

    curr_pos = curr_pos[pdist_argmin]

    before_sort = np.array(marker_pos3d_arr[i][1:])

    marker_pos3d_arr[i][1:] = curr_pos.tolist()

    after_sort = np.array(marker_pos3d_arr[i][1:])

    print(f"{(before_sort - after_sort).max(axis=0)} --> {pdist_argmin}")

    prev_pos = curr_pos.copy()

plt.figure()

color_arr = ['#ff0000', '#ff8000', '#ffff00', '#00ff00', '#00ffff', '#0000ff', '#ff00ff', '#808080']

for data_arr in marker_pos3d_arr:
    tstamp, pos_data = data_arr[0], data_arr[1:]
    pos_data = np.array(pos_data)

    plt.scatter(pos_data[:, 1], pos_data[:, 2], alpha=0.5, s=1, c=color_arr)

plt.show()

# ### Sort rest of data Points ###
# i = 1
# for line in data:
#     if i > 999:
#         break
#     dataSort.append([])
#     # Enter timestamp to dataSort
#     dataSort[i].append(data[i][0])
#     IdxUsed = []
    
#     # Search for P1, enter to dataSort
#     dist = []
#     dataSort[i].append([])
#     j = 0
#     for entry in data[i]:                           # Calculate difference between current point and P1(t=0)
#         if j >= 1:
#             dist.append(absDist(data[i][j], dataSort[0][1]))
#         j += 1
#     dist_sort = []                                  # sort and index differences
#     for index in range(len(dist)):
#         dist_sort.append([dist[index],index])
#     dist_sort.sort()
#     sort_index = []
#     for x in dist_sort:
#         sort_index.append(x[1]+1)
#     # dataSort[i].append(data[i][sort_index[0]])
#     if (absDist(data[i][sort_index[0]],dataSort[0][1]) < 0.004):
#         dataSort[i][1] = (data[i][sort_index[0]])       # write to dataSort
#         IdxUsed.append(sort_index[0])
#     else:
#         dataSort[i][1] = [None]*3
#         print("timestamp " + str(dataSort[i][0][0]) + ": " + "P1 not found")
#         # i += 1
#         # continue
    
#     # Search for P2, enter to dataSort
#     dist = []
#     dataSort[i].append([])
#     j = 0
#     for entry in data[i]:                           # Calculate difference between current point and P2(t=0)
#         if j >= 1:
#             dist.append(absDist(data[i][j], dataSort[0][2]))
#         j += 1
#     dist_sort = []                                  # sort and index differences
#     for index in range(len(dist)):
#         dist_sort.append([dist[index],index])
#     dist_sort.sort()
#     sort_index = []
#     for x in dist_sort:
#         sort_index.append(x[1]+1)
#     k = 0
#     for k in range(len(sort_index)):                           # write to dataSort
#         if (not sort_index[k] in IdxUsed) and (absDist(data[i][sort_index[k]], dataSort[0][2]) < 0.014):
#             # dataSort[i].append(data[i][sort_index[0]])
#             dataSort[i][2] = (data[i][sort_index[k]])
#             IdxUsed.append(sort_index[k])
#             break
#         # else:
#         #     print("timestamp " + str(dataSort[i][0][0]) + ": " + "P1 not found")
#         # k += 1
#     if dataSort[i][2] == []:
#         dataSort[i][2] = [None]*3
#         print("timestamp " + str(dataSort[i][0][0]) + ": " + "P2 not found")
    
#     # Search for P3, enter to dataSort
#     dataSort[i].append([])
#     j = 0
#     for entry in data[i]:                           # compare current points with inital P1
#         if (j >= 1) and (not j in IdxUsed) and (0.018 < absDist(data[i][j], dataSort[0][2]) < 0.041):
#             # dataSort[i].append(data[i][j])
#             dataSort[i][3] = (data[i][j])
#             IdxUsed.append(j)
#             break
#         j += 1
#     if dataSort[i][3] == []:
#         dataSort[i][3] = [None]*3
#         print("timestamp " + str(dataSort[i][0][0]) + ": " + "P3 not found")
#         # i += 1
#         # continue
    
#     # Search for P4, enter to dataSort
#     dataSort[i].append([])
#     j = 0
#     for entry in data[i]:
#         comp = i
#         if dataSort[i][3] == [None]*3:      # define comparison data depending on P3 is available at this timestep
#             while dataSort[comp][3] == [None]*3:
#                 comp -= 1
#         # else:
#         #     comp = i
#         if (j >= 1) and (not j in IdxUsed) and (0.026 < absDist(data[i][j], dataSort[comp][3]) < 0.041):
#             # dataSort[i].append(data[i][j])
#             dataSort[i][4] = (data[i][j])
#             IdxUsed.append(j)
#             break
#         j += 1
#     if dataSort[i][4] == []:
#         dataSort[i][4] = [None]*3
#         print("timestamp " + str(dataSort[i][0][0]) + ": " + "P4 not found")
#         # i += 1
#         # continue
    
#     # Search for P5, enter to dataSort
#     dataSort[i].append([])
#     j = 0
#     for entry in data[i]:
#         comp = i
#         if dataSort[i][4] == [None]*3:      # define comparison data depending on P4 is available at this timestep
#             while dataSort[comp][4] == [None]*3:
#                 comp -= 1
#         if (j >= 1) and (not j in IdxUsed) and (0.005 < absDist(data[i][j], dataSort[comp][4]) < 0.020):
#             # dataSort[i].append(data[i][j])
#             dataSort[i][5] = (data[i][j])
#             IdxUsed.append(j)
#             break
#         j += 1
#     if dataSort[i][5] == []:
#         dataSort[i][5] = [None]*3
#         print("timestamp " + str(dataSort[i][0][0]) + ": " + "P5 not found")
#         # i += 1
#         # continue
    
#     # Search for P6, enter to dataSort
#     dataSort[i].append([])
#     j = 0
#     for entry in data[i]:
#         comp5 = i
#         if dataSort[comp5][5] == [None]*3:      # define comparison data depending on P5 is available at this timestep
#             while dataSort[comp5][5] == [None]*3:
#                 comp5 -= 1
#         comp6 = i-1
#         if dataSort[comp6][6] == [None]*3:
#             while dataSort[comp6][6] == [None]*3:
#                 comp6 -= 1
#         if (j >= 1) and (not j in IdxUsed) and (0.019 < absDist(data[i][j], dataSort[comp5][5]) < 0.029) and (absDist(data[i][j], dataSort[comp6][6]) < 0.02):
#             # dataSort[i].append(data[i][j])
#             dataSort[i][6] = (data[i][j])
#             IdxUsed.append(j)
#             break
#         j += 1
#     if dataSort[i][6] == []:
#         dataSort[i][6] = [None]*3
#         print("timestamp " + str(dataSort[i][0][0]) + ": " + "P6 not found")
#         # i += 1
#         # continue
    
#     # Search for P7, enter to dataSort
#     dataSort[i].append([])
#     j = 0
#     for entry in data[i]:
#         comp = i
#         if dataSort[i][6] == [None]*3:      # define comparison data depending on P4 is available at this timestep
#             while dataSort[comp][6] == [None]*3:
#                 comp -= 1
#     for entry in data[i]:
#         if (j >= 1) and (not j in IdxUsed) and (0.005 < absDist(data[i][j], dataSort[comp][6]) < 0.017):
#             # dataSort[i].append(data[i][j])
#             dataSort[i][7] = (data[i][j])
#             IdxUsed.append(j)
#             break
#         j += 1
#     if dataSort[i][7] == []:
#         dataSort[i][7] = [None]*3
#         print("timestamp " + str(dataSort[i][0][0]) + ": " + "P7 not found")
#         # i += 1
#         # continue
    
#     # Search for P8, enter to dataSort
#     dataSort[i].append([])
#     j = 0
#     for entry in data[i]:
#         comp7 = i
#         if dataSort[i][7] == [None]*3:      # define comparison data depending on P4 is available at this timestep
#             while dataSort[comp7][7] == [None]*3:
#                 comp7 -= 1
#         comp8 = i-1
#         if dataSort[comp8][8] == [None]*3:
#             while dataSort[comp8][8] == [None]*3:
#                 comp8 -= 1
#     for entry in data[i]:
#         # if (j >= 1) and (not j in IdxUsed) and (0.008 < absDist(data[i][j], dataSort[comp7][7]) < 0.024):
#         if (j >= 1) and (not j in IdxUsed) and (0.008 < absDist(data[i][j], dataSort[comp7][7]) < 0.024) and (absDist(data[i][j], dataSort[comp8][8]) < 0.02):
#             # dataSort[i].append(data[i][j])
#             dataSort[i][8] = (data[i][j])
#             IdxUsed.append(j)
#             break
#         j += 1
#     if dataSort[i][8] == []:
#         dataSort[i][8] = [None]*3
#         print("timestamp " + str(dataSort[i][0][0]) + ": " + "P8 not found")
#         i += 1
#         continue
    
#     i += 1
            



# # Rearrange data to coordinate lists / Plot Trajectory
# x = []; y = []; z=[]
# j = 0
# while j < len(dataSort)-1:
#     x.append([])
#     y.append([])
#     z.append([])
#     i = 0
#     # for line in data:
#     #     x[j].append(data[i][j+1][0])
#     #     y[j].append(data[i][j+1][1])
#     #     z[j].append(data[i][j+1][2])
#     #     i += 1
#     for line in dataSort:
#         x[j].append(dataSort[i][j+1][0])
#         y[j].append(dataSort[i][j+1][1])
#         z[j].append(dataSort[i][j+1][2])
#         i += 1
#     # plt.plot(y[0], z[0])
#     # plt.plot(y[1], z[1])
#     # plt.plot(y[4], z[4])
#     plt.plot(y[j], z[j], linestyle='-', marker='x')
#     j += 1
# plt.xlim([0,0.3])
# plt.ylim([-0.1,0.1])
# plt.legend(('P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8'))
# plt.show()