from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math


### Convert list rad > deg
def RadDeg(angRad):
    angDeg = []
    for entry in angRad:
        angDeg.append(math.degrees(entry))
    return angDeg


### Import data ###
path = str(Path(__file__).parent)

print("Which file should be analized?")
# fileName=input()
fileName="test_10000"

directory_path = 'QTM_Data/' + fileName + '.tsv'


### Convert data to dictionary ###
dataImport = pd.read_csv (directory_path, sep = '\t', skiprows=11, header=None)
dataDict = {
    "timestamp": [],    ## timestamps
    "MC": [],           ## positions of markers
    "PP": [],
    "MP": [],
    "DP": [],
    "angMC": [],        ## angles of phalanges [rad]    ∠ = arctan(Δz/Δy)
    "angPP": [],
    "angMP": [],
    "angDP": []
    }
    ## form of dataDict point entries: dataDict["phalanx"][timestep][markerID][x/y/z]

i = 0
for line in dataImport[0]:
    ## timestamps
    dataDict["timestamp"].append(dataImport[1][i])
    
    ## points
    dataDict["MC"].append([])    
    for j in range(2,11):
        if (j-2)%3 == 0:                # generate new list for every x coordinate
            dataDict["MC"][i].append([])
        ## write MC coordinates
        if j in range(2,5):             # write MC0 coordinates
            dataDict["MC"][i][0].append(dataImport[j][i])
        elif j in range(5,8):           # write MC1 coordinates
            dataDict["MC"][i][1].append(dataImport[j][i])
        elif j in range(8,11):          # write MC2 coordinates
            dataDict["MC"][i][2].append(dataImport[j][i])
    
    dataDict["PP"].append([])
    for j in range(11,20):
        if (j-2)%3 == 0:
            dataDict["PP"][i].append([])
        ## write PP coordinates
        if j in range(11,14):
            dataDict["PP"][i][0].append(dataImport[j][i])
        elif j in range(14,17):
            dataDict["PP"][i][1].append(dataImport[j][i])
        elif j in range(17,20):
            dataDict["PP"][i][2].append(dataImport[j][i])
    
    dataDict["MP"].append([])
    for j in range(20,29):
        if (j-2)%3 == 0:
            dataDict["MP"][i].append([])
        ## write MP coordinates
        if j in range(10,23):
            dataDict["MP"][i][0].append(dataImport[j][i])
        elif j in range(23,26):
            dataDict["MP"][i][1].append(dataImport[j][i])
        elif j in range(26,29):
            dataDict["MP"][i][2].append(dataImport[j][i])
    
    dataDict["DP"].append([])
    for j in range(29,38):
        if (j-2)%3 == 0:
            dataDict["DP"][i].append([])
        ## write DP coordinates
        if j in range(29,32):
            dataDict["DP"][i][0].append(dataImport[j][i])
        elif j in range(32,35):
            dataDict["DP"][i][1].append(dataImport[j][i])
        elif j in range(35,38):
            dataDict["DP"][i][2].append(dataImport[j][i])
    
    ## angles
    dataDict["angMC"].append(math.atan2(dataDict["MC"][i][1][2]-dataDict["MC"][i][2][2], dataDict["MC"][i][1][1]-dataDict["MC"][i][2][1]))
    dataDict["angPP"].append(math.atan2(dataDict["PP"][i][1][2]-dataDict["PP"][i][0][2], dataDict["PP"][i][1][1]-dataDict["PP"][i][0][1]))
    dataDict["angMP"].append(math.atan2(dataDict["MP"][i][1][2]-dataDict["MP"][i][0][2], dataDict["MP"][i][1][1]-dataDict["MP"][i][0][1]))
    dataDict["angDP"].append(math.atan2(dataDict["DP"][i][1][2]-dataDict["DP"][i][0][2], dataDict["DP"][i][1][1]-dataDict["DP"][i][0][1]))
    
    if i % 6000 == 0:
        print("dictionary conversion:"+str(i))
    
    i += 1
    

### convert data to x/y/z sublists, move origin to MC0 ###
i = 0
MC0 = [[] for _ in range(3)]
MC1 = [[] for _ in range(3)]
MC2 = [[] for _ in range(3)]
PP0 = [[] for _ in range(3)]
PP1 = [[] for _ in range(3)]
PP2 = [[] for _ in range(3)]
MP0 = [[] for _ in range(3)]
MP1 = [[] for _ in range(3)]
MP2 = [[] for _ in range(3)]
DP0 = [[] for _ in range(3)]
DP1 = [[] for _ in range(3)]
DP2 = [[] for _ in range(3)]

for line in dataDict["MC"]:
    for j in range(3):
        # MC0[j].append(dataDict["MC"][i][0][j])
        # MC1[j].append(dataDict["MC"][i][1][j])
        # MC2[j].append(dataDict["MC"][i][2][j])        
        # PP0[j].append(dataDict["PP"][i][0][j])
        # PP1[j].append(dataDict["PP"][i][1][j])
        # PP2[j].append(dataDict["PP"][i][2][j])
        # MP0[j].append(dataDict["MP"][i][0][j])
        # MP1[j].append(dataDict["MP"][i][1][j])
        # MP2[j].append(dataDict["MP"][i][2][j])        
        # DP0[j].append(dataDict["DP"][i][0][j])
        # DP1[j].append(dataDict["DP"][i][1][j])
        # DP2[j].append(dataDict["DP"][i][2][j])
        MC0[j].append(dataDict["MC"][i][0][j] - dataDict["MC"][0][0][j])
        MC1[j].append(dataDict["MC"][i][1][j] - dataDict["MC"][0][0][j])
        MC2[j].append(dataDict["MC"][i][2][j] - dataDict["MC"][0][0][j])        
        PP0[j].append(dataDict["PP"][i][0][j] - dataDict["MC"][0][0][j])
        PP1[j].append(dataDict["PP"][i][1][j] - dataDict["MC"][0][0][j])
        PP2[j].append(dataDict["PP"][i][2][j] - dataDict["MC"][0][0][j])
        MP0[j].append(dataDict["MP"][i][0][j] - dataDict["MC"][0][0][j])
        MP1[j].append(dataDict["MP"][i][1][j] - dataDict["MC"][0][0][j])
        MP2[j].append(dataDict["MP"][i][2][j] - dataDict["MC"][0][0][j])        
        DP0[j].append(dataDict["DP"][i][0][j] - dataDict["MC"][0][0][j])
        DP1[j].append(dataDict["DP"][i][1][j] - dataDict["MC"][0][0][j])
        DP2[j].append(dataDict["DP"][i][2][j] - dataDict["MC"][0][0][j])
    i += 1


### plot trajectory ###
plt.figure(0)
plt.plot([MC0[1],MC1[1]], [MC0[2],MC1[2]], color='blue', alpha=0.05)
plt.plot([MC2[1],MC1[1]], [MC2[2],MC1[2]], color='blue', alpha=0.05)
plt.plot([PP0[1],PP1[1]], [PP0[2],PP1[2]], color='green', alpha=0.05)
plt.plot([MP0[1],MP1[1]], [MP0[2],MP1[2]], color='yellow', alpha=0.05)
plt.plot([DP0[1],DP1[1]], [DP0[2],DP1[2]], color='red', alpha=0.05)
## empty fake plots for legend
plt.plot(np.NaN, np.NaN, color='blue', label='MC')
plt.plot(np.NaN, np.NaN, color='green', label='PP')
plt.plot(np.NaN, np.NaN, color='yellow', label='MP')
plt.plot(np.NaN, np.NaN, color='red', label='DP')

plt.axis('equal')
plt.title("Trajectories")
plt.xlabel("y [mm]")
plt.ylabel("z [mm]")
plt.legend()
plt.show()


# ### plot z(t) of DP1
# plt.figure(1)
# plt.plot(dataDict["timestamp"], DP1[1])
# plt.plot(dataDict["timestamp"], DP1[2])
# plt.xlabel("t [s]")
# plt.ylabel("y/z [mm]")
# plt.show


### plot α(t) of MP
plt.figure(2)
plt.plot(dataDict["timestamp"], RadDeg(dataDict["angMP"]))
plt.title("α MP")
plt.xlabel("t [s]")
plt.ylabel("α [°]")
plt.show



### search for max flexion/extension time