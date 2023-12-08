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
import os
import json

path = str(Path(__file__).parent)

print("Which file should be analized?")
# fileName=input()
fileName="test4_50g"




def absCoord(list3D):
    return math.sqrt(list3D[0]**2 + list3D[1]**2 + list3D[2]**2)

def absDist(list1, list2):
    # return math.sqrt((list1[0]-list2[0])**2 + (list1[1]-list2[1])**2 + (list1[2]-list2[2])**2)
    return math.sqrt((list1[1]-list2[1])**2 + (list1[2]-list2[2])**2)



### Inspired by ChatGPT ###

# import file
# with open('ExportData/'+fileName+'.txt') as file:
#     header = file.readlines()[0]
# with open('ExportData/'+fileName+'.txt') as file:
#     dataStr = file.readlines()[1:]



# Function to read JSON files and merge data
def merge_json_files(directory):
    combined_data = {"timestamp": [], "points": []}

    # Get a list of JSON files in the directory
    json_files = [file for file in os.listdir(directory)] #if file.startswith(fileName) and file.endswith(".json")]
    json_files.sort()  # Sort the files if needed

    for file in json_files:
        with open(os.path.join(directory, file), 'r') as json_file:
            data = json.load(json_file)
            combined_data["timestamp"].extend(data["timestamp"])
            combined_data["points"].extend(data["points"])

    return combined_data

# Replace 'directory_path' with the path to your JSON files' directory
directory_path = 'ExportData/' + fileName
tPosData = merge_json_files(directory_path)

dataSort = {"timestamp": [], "points": []}

i = 1900
for x in range(i):
    # dataSort["timestamp"].append(tPosData["timestamp"][i])
    dataSort["points"].append([[None]*3]*8)
# for timestep in tPosData["points"]:
for timestep in range(100):
    
    dataSort["timestamp"].append(tPosData["timestamp"][i])
    dataSort["points"].append([None]*8)

    # calculate relative distances between all points of timestep
    distMat = np.empty((8,8), float)
    j = 0
    for point in tPosData["points"][i]:
        k = 0
        for pCompar in tPosData["points"][i]:
            distMat[j,k] = absDist(tPosData["points"][i][j], tPosData["points"][i][k])
            k += 1
        j += 1
    
    
    # search for DP markers (distance = 14.5 mm, points 6 & 7)
    distMat67 = abs(distMat - 0.0145)
    couple67 = np.where(distMat67 == np.min(distMat67))[0]
    dataSort["points"][i][6] = tPosData["points"][i][couple67[0]]
    dataSort["points"][i][7] = tPosData["points"][i][couple67[1]]
    
    # delete P6/7 from distMat
    distMat_del = np.copy(distMat)
    distMat_del[couple67[0]].fill(None)
    distMat_del[couple67[1]].fill(None)
    distMat_del[:, couple67[0]].fill(None)
    distMat_del[:, couple67[1]].fill(None)
    
    
    # search for MP markers (distance = 24 mm, points 4 & 5)
    distMat45 = abs(distMat_del - 0.024)
    couple45 = np.where(distMat45 == np.nanmin(distMat45))[0]
    dataSort["points"][i][4] = tPosData["points"][i][couple45[0]]
    dataSort["points"][i][5] = tPosData["points"][i][couple45[1]]
    
    # delete P4/5 from distMat
    # distMat_0123 = np.copy(distMat_012345)
    distMat_del[couple45[0]].fill(None)
    distMat_del[couple45[1]].fill(None)
    distMat_del[:, couple45[0]].fill(None)
    distMat_del[:, couple45[1]].fill(None)
    
    
    # search for MC markers (distance = 8 mm, points 0 & 1)
    distMat01 = abs(distMat_del - 0.008)
    couple01 = np.where(distMat01 == np.nanmin(distMat01))[0]
    dataSort["points"][i][0] = tPosData["points"][i][couple01[0]]
    dataSort["points"][i][1] = tPosData["points"][i][couple01[1]]
    
    # delete P0/1 from distMat
    # distMat_23 = np.copy(distMat_0123)
    distMat_del[couple01[0]].fill(None)
    distMat_del[couple01[1]].fill(None)
    distMat_del[:, couple01[0]].fill(None)
    distMat_del[:, couple01[1]].fill(None)
    
    
    # search for PP markers (distance = 36 mm, points 2 & 3)
    distMat23 = abs(distMat_del - 0.036)
    couple23 = np.where(distMat23 == np.nanmin(distMat23))[0]
    dataSort["points"][i][2] = tPosData["points"][i][couple23[0]]
    dataSort["points"][i][3] = tPosData["points"][i][couple23[1]]
    
    # delete P0/1 from distMat
    # distMat_ = np.copy(distMat_23)
    distMat_del[couple23[0]].fill(None)
    distMat_del[couple23[1]].fill(None)
    distMat_del[:, couple23[0]].fill(None)
    distMat_del[:, couple23[1]].fill(None)
    
    
    i += 1



print()
# # Access the combined data
# print(combined_json_data)



# # convert imported file to float lists
# data = []
# i = 0
# for line in dataStr:                                                            # Remove line break, split list in data points
#     dataLine = line.strip()[1:-1].split('], [')
#     i += 1
#     dataTimestep = []
#     for entry in dataLine:                                                      # Convert entries of single lines to float
#         values = entry.replace('[', '').replace(']', '').split(', ')
#         values = [float(value) for value in values if value.strip() != '']
#         dataTimestep.append(values)
#     data.append(dataTimestep)                                                   # Combine data lines to list
    
    
# print(data[0])
# print(data[0][0])
# print(data[0][1])
# print(data[0][1][0])




# ### Sort first data points at t=0 ###
# dist0 = []
# j = 0
# for entry in data[0]:                           # Calculate absolute value of time 0 coordinates
#     if j >= 1:
#         dist0.append(absCoord(entry))
#     j += 1

# dist0_sort = []                                 # sort and index absolute values
# for index in range(len(dist0)):
#     dist0_sort.append([dist0[index],index])
# dist0_sort.sort()
# sort_index = []
# for x in dist0_sort:
#     sort_index.append(x[1]+1)
        
# dataSort = [[]]                                 # write list with sorted values
# j = 0
# for entry in data[0]:
#     if j == 0:
#         dataSort[0].append(data[0][0])
#     else:
#         dataSort[0].append(data[0][sort_index[j-1]])
#     j += 1


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
            



# Rearrange data to coordinate lists / Plot Trajectory
x = []; y = []; z=[]
j = 0
# while j < len(dataSort)-1:
# while j < len(data[0])-1:
while j < len(tPosData["points"][0]):
    x.append([])
    y.append([])
    z.append([])
    i = 0
    # for line in data:
    #     x[j].append(data[i][j+1][0])
    #     y[j].append(data[i][j+1][1])
    #     z[j].append(data[i][j+1][2])
    #     i += 1
    # for line in dataSort:
    #     x[j].append(dataSort[i][j+1][0])
    #     y[j].append(dataSort[i][j+1][1])
    #     z[j].append(dataSort[i][j+1][2])
    #     i += 1
    # for timestep in tPosData["points"]:
    #     x[j].append(tPosData["points"][i][j][0])
    #     y[j].append(tPosData["points"][i][j][1])
    #     z[j].append(tPosData["points"][i][j][2])
    #     i += 1
    for timestep in dataSort["points"]:
        x[j].append(dataSort["points"][i][j][0])
        y[j].append(dataSort["points"][i][j][1])
        z[j].append(dataSort["points"][i][j][2])
        i += 1
    
    # plt.plot(y[1], z[1], linestyle='', marker='.', alpha=0.1)    
    plt.plot(y[j], z[j], linestyle='-', marker='.', label='P'+str(j), alpha=0.1)
    # patches.append(mpatches.Patch())
    j += 1
plt.axis('equal')
# plt.xlim([0,0.3])
# plt.ylim([-0.1,0.1])
# plt.legend(['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8'])
plt.xlabel("y [m]")
plt.ylabel("z [m]")
plt.legend()
plt.show()