from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math
import copy


### Convert list rad > deg ###
def RadDeg(angRad):
    angDeg = []
    for entry in angRad:
        angDeg.append(math.degrees(entry))
    return angDeg


### Calculate distance between 2 points (y/z-coordinates) ###
def dist(point1, point2):
    return math.sqrt((point1[1]-point2[1])**2 + (point1[2]-point2[2])**2)


### Moving average per minute ###
def movAvg(arr):
    winSize = 60                        # nr of items to calc avg (60 for average of min in s)
    pdSer = pd.Series(arr)
    windows = pdSer.rolling(winSize)
    movAvgs = windows.mean()
    movAvgList = movAvgs.tolist()
    return movAvgList#[winSize-1:]


### Extraction of reference_timestamp out of tsv file (ChatGPT) ###
def extract_reference_timestamp(file_path):
    with open(file_path, 'r') as file:
        # jump to 8th line
        for _ in range(7):
            next(file)
        # extract reference_timestamp
        reference_timestamp = pd.to_datetime(next(file).split('\t')[1], format='%Y-%m-%d, %H:%M:%S.%f').timestamp()
    return reference_timestamp


### Process tsv file (ChatGPT) ###
def process_tsv_file(file_path, dataDict, markOrd, prev_timestamp):
    dataImport = pd.read_csv(file_path, sep='\t', skiprows=11, header=None)        
    reference_timestamp = extract_reference_timestamp(file_path)    # extract reference_timestamp      
    for i in range(len(dataImport[0])):   # iterate through tsv file
        timestamp = dataImport[1][i]
        timestamp += reference_timestamp  # Add reference timestamp
        dataDict["timestamp"].append(timestamp)
    return dataDict, timestamp


### Export plot as PDF ###
def ExportPDF(nameAttachment):
    ExportPath = 'C:/Users/samue/polybox/ETH/Master/Master Thesis/images/plots/'
    plt.savefig(ExportPath + fileName + "_" + nameAttachment + ".pdf",
                bbox_inches='tight', pad_inches=0, transparent=1)
    
    
# %%
### Import data ###
# path = str(Path(__file__).parent)
# path = str(Path(__file__).parent.parent.parent)+'/QTM/Data/'
print("Data folder path (Default: C:/Users/samue/polybox/ETH/Master/Master Thesis/QTM/Data/):")
path = input() or 'C:/Users/samue/polybox/ETH/Master/Master Thesis/QTM/Data/'
print("Which file should be analized?")
fileName = input()
print("Number of files?")
numFiles = int(input())
print('\n')

# directory_path = 'QTM_Data/' + fileName + '.tsv'
# directory_path = path + fileName + '.tsv'
directory_path = path + fileName + '_{:04d}.tsv'  # Dateinamen-Vorlage
#%%


### Create empty dictionary for data ###
start_timestamp = 0
start_reference_timestamp = extract_reference_timestamp(directory_path.format(1))
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

dataDict["angMC"] = []
dataDict["angPP"] = []
dataDict["angMP"] = []
dataDict["angDP"] = []


i = 0
### Iterate through files (0001-numFiles) (ChatGPT) ###
for fileNum in range(1, numFiles+1):
    
    dataImport = pd.read_csv(directory_path.format(fileNum), sep = '\t', skiprows=11, header=None)
    
    ### Read out order of markers ###
    markOrd = pd.read_csv (directory_path.format(fileNum), sep = '\t', skiprows=9, nrows=0)#, header=None)
    markOrd = markOrd.drop(columns=['MARKER_NAMES'])
    markOrd.loc[0] = range(len(markOrd.columns))

    current_file_path = directory_path.format(fileNum)
    dataDict, start_timestamp = process_tsv_file(current_file_path, dataDict, markOrd, start_timestamp)

    #%%
    
### Write data in dictionary ###    
    # i = 0
    iFile = 0
    for line in dataImport[0]:
        ## timestamps
        # dataDict["timestamp"].append(dataImport[1][i])
        dataDict["timestamp"][i] = round(dataDict["timestamp"][i] - start_reference_timestamp,2)
        
        ## points
        dataDict["MC"].append([])           
        for j in range(2,11):
            if (j-2)%3 == 0:                # generate new list for every x coordinate
                k = 0 
                dataDict["MC"][i].append([])
            ## write MC coordinates
            if j in range(2,5):             # write MC0 coordinates
                dataDict["MC"][i][0].append(dataImport[2+markOrd.loc[0,'MC0']*3+k][iFile])
            elif j in range(5,8):           # write MC1 coordinates
                dataDict["MC"][i][1].append(dataImport[2+markOrd.loc[0,'MC1']*3+k][iFile])
            elif j in range(8,11):          # write MC2 coordinates
                dataDict["MC"][i][2].append(dataImport[2+markOrd.loc[0,'MC2']*3+k][iFile])
            k += 1
        
        dataDict["PP"].append([])
        for j in range(11,20):
            if (j-2)%3 == 0:
                k = 0
                dataDict["PP"][i].append([])
            ## write PP coordinates
            if j in range(11,14):
                dataDict["PP"][i][0].append(dataImport[2+markOrd.loc[0,'PP0']*3+k][iFile])
            elif j in range(14,17):
                dataDict["PP"][i][1].append(dataImport[2+markOrd.loc[0,'PP1']*3+k][iFile])
            elif j in range(17,20):
                dataDict["PP"][i][2].append(dataImport[2+markOrd.loc[0,'PP2']*3+k][iFile])
            k += 1
        
        dataDict["MP"].append([])
        for j in range(20,29):
            if (j-2)%3 == 0:
                k = 0
                dataDict["MP"][i].append([])
            ## write MP coordinates
            if j in range(10,23):
                dataDict["MP"][i][0].append(dataImport[2+markOrd.loc[0,'MP0']*3+k][iFile])
            elif j in range(23,26):
                dataDict["MP"][i][1].append(dataImport[2+markOrd.loc[0,'MP1']*3+k][iFile])
            elif j in range(26,29):
                dataDict["MP"][i][2].append(dataImport[2+markOrd.loc[0,'MP2']*3+k][iFile])
            k += 1
        
        dataDict["DP"].append([])
        for j in range(29,38):
            if (j-2)%3 == 0:
                k = 0
                dataDict["DP"][i].append([])
            ## write DP coordinates
            if j in range(29,32):
                dataDict["DP"][i][0].append(dataImport[2+markOrd.loc[0,'DP0']*3+k][iFile])
            elif j in range(32,35):
                dataDict["DP"][i][1].append(dataImport[2+markOrd.loc[0,'DP1']*3+k][iFile])
            elif j in range(35,38):
                dataDict["DP"][i][2].append(dataImport[2+markOrd.loc[0,'DP2']*3+k][iFile])
            k += 1
        
        ## angles
        dataDict["angMC"].append(np.arctan2(dataDict["MC"][i][1][2]-dataDict["MC"][i][2][2], dataDict["MC"][i][1][1]-dataDict["MC"][i][2][1]))
        dataDict["angPP"].append(np.arctan2(dataDict["PP"][i][1][2]-dataDict["PP"][i][0][2], dataDict["PP"][i][1][1]-dataDict["PP"][i][0][1]))
        dataDict["angMP"].append(np.arctan2(dataDict["MP"][i][1][2]-dataDict["MP"][i][0][2], dataDict["MP"][i][1][1]-dataDict["MP"][i][0][1]))
        dataDict["angDP"].append(np.arctan2(dataDict["DP"][i][1][2]-dataDict["DP"][i][0][2], dataDict["DP"][i][1][1]-dataDict["DP"][i][0][1]))
        
        if i % 1000 == 0:
            print(end='\x1b[2K')    # ANSI code to clear line
            print("Save data in dictionary: "+str(round(i/(numFiles*len(dataImport[0]))*100, 1))+" %", end='\r')
        
        i += 1
        iFile += 1
print(end='\x1b[2K')    # ANSI code to clear line
print("Save data in dictionary: "+str(round(i/(numFiles*len(dataImport[0]))*100, 1))+" %", end='\n')
# print('\n')


#%%

### Calculate recording frequency ###
freq = int(1 / (dataDict["timestamp"][1] - dataDict["timestamp"][0]))


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
        # MC0[j].append(dataDict["MC"][i][0][j])                            # original position
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
        MC0[j].append(dataDict["MC"][i][0][j] - dataDict["MC"][i][0][j])    # relative position to MC0
        MC1[j].append(dataDict["MC"][i][1][j] - dataDict["MC"][i][0][j])
        MC2[j].append(dataDict["MC"][i][2][j] - dataDict["MC"][i][0][j])        
        PP0[j].append(dataDict["PP"][i][0][j] - dataDict["MC"][i][0][j])
        PP1[j].append(dataDict["PP"][i][1][j] - dataDict["MC"][i][0][j])
        PP2[j].append(dataDict["PP"][i][2][j] - dataDict["MC"][i][0][j])
        MP0[j].append(dataDict["MP"][i][0][j] - dataDict["MC"][i][0][j])
        MP1[j].append(dataDict["MP"][i][1][j] - dataDict["MC"][i][0][j])
        MP2[j].append(dataDict["MP"][i][2][j] - dataDict["MC"][i][0][j])        
        DP0[j].append(dataDict["DP"][i][0][j] - dataDict["MC"][i][0][j])
        DP1[j].append(dataDict["DP"][i][1][j] - dataDict["MC"][i][0][j])
        DP2[j].append(dataDict["DP"][i][2][j] - dataDict["MC"][i][0][j])
    i += 1
    if i % 1000 == 0:
        print(end='\x1b[2K')    # ANSI code to clear line
        print("Convert data to x/y/z sublists: "+str(round(i/len(dataDict["MC"])*100, 1))+" %", end='\r')
# print(end='\x1b[2K')    # ANSI code to clear line
print("Convert data to x/y/z sublists: "+str(round(i/len(dataDict["MC"])*100, 1))+" %", end='\n')

# %%

### Remove data before movement starts/after stop (α MP < 3°)
befMov = []
i = 0
while abs(math.degrees(dataDict["angMP"][i] - dataDict["angMP"][0])) < 3:
    befMov.append(i)
    i += 1
tStartIdx = max(befMov)+1
tStart = dataDict["timestamp"][tStartIdx]

aftMov = []
i = len(dataDict["timestamp"])-1
while abs(math.degrees(dataDict["angMP"][i] - dataDict["angMP"][-1])) < 3:
    aftMov.append(i)
    i -= 1
tStopIdx = min(aftMov)-1
tStop = dataDict["timestamp"][tStopIdx]

tShift = dataDict["timestamp"][tStartIdx:tStopIdx]
tShift = [x - dataDict["timestamp"][befMov[-1]] for x in tShift]
movIdx = [*range(tStartIdx,tStopIdx)]
print("Remove stationary data: 100.0 %")


### Write biggest MP angle of every second to dict ###
angMax = {
    "frame": [],
    "timestamp": [],
    "PP": [],
    "MP": [],
    "DP": []
    }
i = 0
iMax_check = []
for t_ in np.arange(tStart, tStop-1, 1):
    if round(t_,2) in dataDict["timestamp"]:
        i_ = dataDict["timestamp"].index(round(t_,2))
        iMax = i_ + np.argmax(dataDict["angMP"][i_:i_+freq])
        angMax["frame"].append(iMax)
        angMax["timestamp"].append(dataDict["timestamp"][iMax])
        angMax["PP"].append(dataDict["angPP"][iMax])
        angMax["MP"].append(dataDict["angMP"][iMax])
        angMax["DP"].append(dataDict["angDP"][iMax])
    i += 1
    print(end='\x1b[2K')    # ANSI code to clear line
    print("Calculate max angles: "+str(round(i/len(np.arange(tStart, tStop-1, 1))*100, 1))+" %", end='\r')
print("Calculate max angles: "+str(round(i/len(np.arange(tStart, tStop-1, 1))*100, 1))+" %", end='\n')


### Write lowest MP angle of every second to dict ###
angMin = {
    "frame": [],
    "timestamp": [],
    "PP": [],
    "MP": [],
    "DP": []
    }
i = 0
for t_ in np.arange(tStart, tStop-1, 1):
    if round(t_,2) in dataDict["timestamp"]:
        i_ = dataDict["timestamp"].index(round(t_,2))
        iMin = i_ + np.argmin(dataDict["angMP"][i_:i_+freq])
        angMin["frame"].append(iMin)
        angMin["timestamp"].append(dataDict["timestamp"][iMin])
        angMin["PP"].append(dataDict["angPP"][iMin])
        angMin["MP"].append(dataDict["angMP"][iMin])
        angMin["DP"].append(dataDict["angDP"][iMin])
    i += 1
    print(end='\x1b[2K')    # ANSI code to clear line
    print("Calculate min angles: "+str(round(i/len(np.arange(tStart, tStop-1, 1))*100, 1))+" %", end='\r')
print("Calculate min angles: "+str(round(i/len(np.arange(tStart, tStop-1, 1))*100, 1))+" %", end='\n')


# %%
### DP1 speed/acceleration (+: flexion, -: extension) ###
i = 1
speedDP1 = [0]
accelDP1 = [0]
for line in dataDict["timestamp"]:
    if i >= len(dataDict["timestamp"]):
        break
    dt = dataDict["timestamp"][i] - dataDict["timestamp"][i-1]
    dDist = dist(dataDict["DP"][i][1], dataDict["DP"][i-1][1])/1000                 # [dist_dt] = m
    dDist = dDist * np.sign(dataDict["DP"][i][1][2] - dataDict["DP"][i-1][1][2])    # sign: + if z inc, - if z dec
    speed_dt = dDist/dt
    speedDP1.append(speed_dt)
    dSpeed = speedDP1[i]-speedDP1[i-1]
    accel_dt = dSpeed/dt
    accelDP1.append(accel_dt)
    i += 1
    print(end='\x1b[2K')    # ANSI code to clear line
    print("Calculate DP1 speed & acceleration: "+str(round(i/len(dataDict["timestamp"])*100, 1))+" %", end='\r')
print("Calculate DP1 speed & acceleration: "+str(round(i/len(dataDict["timestamp"])*100, 1))+" %", end='\n')
# %%


### plot trajectory ###
print("Plot trajectory")
plt.figure(0)
# a = 0; b = -1               # plot range (full range)
# a = tStartIdx; b = a+1000; step = 10      # plot range (first 10s after start of movement)
a = tStartIdx; b = tStopIdx; step = 70     # plot range (every iteration, but only every second)

# plt.plot([MC0[1],MC1[1]], [MC0[2],MC1[2]], color='blue', alpha=0.05)
# plt.plot([MC2[1],MC1[1]], [MC2[2],MC1[2]], color='blue', alpha=0.05)
# plt.plot([PP0[1],PP1[1]], [PP0[2],PP1[2]], color='limegreen', alpha=0.05)
# plt.plot([MP0[1],MP1[1]], [MP0[2],MP1[2]], color='gold', alpha=0.05)
# plt.plot([DP0[1],DP1[1]], [DP0[2],DP1[2]], color='red', alpha=0.05)
plt.plot([MC0[1][a:b:step],MC1[1][a:b:step]], [MC0[2][a:b:step],MC1[2][a:b:step]], color='blue', alpha=0.005)
plt.plot([MC2[1][a:b:step],MC1[1][a:b:step]], [MC2[2][a:b:step],MC1[2][a:b:step]], color='blue', alpha=0.005)
plt.plot([PP0[1][a:b:step],PP1[1][a:b:step]], [PP0[2][a:b:step],PP1[2][a:b:step]], color='green', alpha=0.005)
plt.plot([MP0[1][a:b:step],MP1[1][a:b:step]], [MP0[2][a:b:step],MP1[2][a:b:step]], color='gold', alpha=0.005)
plt.plot([DP0[1][a:b:step],DP1[1][a:b:step]], [DP0[2][a:b:step],DP1[2][a:b:step]], color='red', alpha=0.005)
## empty fake plots for legend
plt.plot(np.NaN, np.NaN, color='blue', label='MC')
plt.plot(np.NaN, np.NaN, color='GREEN', label='PP')
plt.plot(np.NaN, np.NaN, color='gold', label='MP')
plt.plot(np.NaN, np.NaN, color='red', label='DP')

plt.axis('equal')
plt.title("Trajectories")
plt.xlabel("y [mm]")
plt.ylabel("z [mm]")
plt.legend()
plt.grid(True)
plt.rc('grid', alpha=0.5, linewidth=0.5)
ExportPDF("trajectory")
plt.show

#%%
# ### plot extreme positions y/z(t) of DP1 ###
# plt.figure(1)
# plt.plot([dataDict["timestamp"][i]-tStart for i in angMax["frame"] if i in movIdx],
#           [DP1[1][i] for i in angMax["frame"] if i in movIdx], label = "y max")
# plt.plot([dataDict["timestamp"][i]-tStart for i in angMax["frame"] if i in movIdx],
#           [DP1[2][i] for i in angMax["frame"] if i in movIdx], label = "z max")
# plt.plot([dataDict["timestamp"][i]-tStart for i in angMin["frame"] if i in movIdx],
#           [DP1[1][i] for i in angMin["frame"] if i in movIdx], label = "y min")
# plt.plot([dataDict["timestamp"][i]-tStart for i in angMin["frame"] if i in movIdx],
#           [DP1[2][i] for i in angMin["frame"] if i in movIdx], label = "z min")
# plt.title("y/z at extreme positions")
# plt.xlabel("t [s]")
# plt.ylabel("y/z [mm]")
# plt.legend()
# plt.show

# plt.figure(2)
# ax = plt.axes(projection='3d')
# t = [dataDict["timestamp"][i] for i in angMax["frame"]]
# y = [DP1[2][i] for i in angMax["frame"]]
# z = [DP1[2][i] for i in angMax["frame"]]
# ax.plot3D(t, y, z)
# ax.set_title('3D line plot geeks for geeks')
# plt.show()


### plot extreme positions z(y) of DP1 ###
print("Plot extreme positions of DP1")
plt.figure(1)
plt.plot([DP1[1][i] for i in angMax["frame"]], [DP1[2][i] for i in angMax["frame"]], color='red', linewidth=0.1)
plt.plot([DP1[1][i] for i in angMin["frame"]], [DP1[2][i] for i in angMin["frame"]], color='crimson', linewidth=0.1)
## start/end points (ChatGPT)
plt.plot(DP1[1][angMax["frame"][0]], DP1[2][angMax["frame"][0]], marker='o', markersize=5, color='red')
plt.plot(DP1[1][angMin["frame"][0]], DP1[2][angMin["frame"][0]], marker='o', markersize=5, color='crimson')
plt.plot(DP1[1][angMax["frame"][-1]], DP1[2][angMax["frame"][-1]], marker='^', markersize=5, color='red')
plt.plot(DP1[1][angMin["frame"][-1]], DP1[2][angMin["frame"][-1]], marker='^', markersize=5, color='crimson')
## empty fake plots for legend
plt.plot(np.NaN, np.NaN, color='red', label='flexed')
plt.plot(np.NaN, np.NaN, color='crimson', label='extended')
plt.plot(np.NaN, np.NaN, marker='o', markersize=5, linestyle='None', color='k', label='first')
plt.plot(np.NaN, np.NaN, marker='^', markersize=5, linestyle='None', color='k', label='last')
plt.title("extreme positions DP1")
plt.xlabel("y [mm]")
plt.ylabel("z [mm]")
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.rc('grid', alpha=0.5, linewidth=0.5)
ExportPDF("DP1_extremePos")
plt.show


### plot distance of DP1 compared to 1st extreme position ###
print("Plot deviation of DP1 extreme positions over time")
plt.figure(2)
plt.plot(angMin["timestamp"]-tStart, [dist(dataDict["DP"][i][1], dataDict["DP"][angMin["frame"][0]][1]) for i in angMin["frame"]], 
         color='crimson', alpha=0.3)#, label="min")
plt.plot(angMax["timestamp"]-tStart, [dist(dataDict["DP"][i][1], dataDict["DP"][angMax["frame"][0]][1]) for i in angMax["frame"]], 
         color='red', alpha=0.3)#, label="max")
plt.plot(angMin["timestamp"]-tStart, movAvg([dist(dataDict["DP"][i][1], dataDict["DP"][angMin["frame"][0]][1]) for i in angMin["frame"]]),
         color='crimson', linestyle=':')#, label="average per minute min")
plt.plot(angMax["timestamp"]-tStart, movAvg([dist(dataDict["DP"][i][1], dataDict["DP"][angMax["frame"][0]][1]) for i in angMax["frame"]]),
         color='red', linestyle=':')#, label="average per minute max")
## legend fake plots
plt.plot(np.nan, np.nan, color='crimson', label="extended")
plt.plot(np.nan, np.nan, color='red', label="flexed")
plt.plot(np.NaN, np.NaN, color='black', linestyle=':', label="avg/min")
plt.title("Deviation of DP1 extreme positions over time")
plt.xlabel("t [s]")
plt.ylabel("deviation [mm]")
plt.legend()
plt.grid(True)
plt.rc('grid', alpha=0.5, linewidth=0.5)
ExportPDF("DP1_extrPosDev")
plt.show


### plot α(t) of MP (first 10s) ###
plt.figure(3)
plt.plot(tShift[0:1000], [math.degrees(dataDict["angPP"][i]) for i in movIdx[0:1000]], alpha=0.5, color='green', label="PP")
plt.plot(tShift[0:1000], [math.degrees(dataDict["angMP"][i]) for i in movIdx[0:1000]], alpha=0.5, color='gold', label="MP")
plt.plot(tShift[0:1000], [math.degrees(dataDict["angDP"][i]) for i in movIdx[0:1000]], alpha=0.5, color='red', label="DP")
plt.title("angles first 10 s")
plt.xlabel("t [s]")
plt.ylabel("α [°]")
plt.legend()
plt.grid(True)
plt.rc('grid', alpha=0.5, linewidth=0.5)
plt.show


### plot α_max/min(t) of MP ###
plt.figure(4)
## flexed
plt.plot(angMax["timestamp"]-tStart, RadDeg(angMax["PP"]), color='limegreen', alpha=0.3)#, label="$α_{max}$")
plt.plot(angMax["timestamp"]-tStart, movAvg(RadDeg(angMax["PP"])),
         color='limegreen', linestyle=':')#, label="$α_{max}$ (average per minute)")
plt.plot(angMax["timestamp"]-tStart, RadDeg(angMax["MP"]), color='gold', alpha=0.3)
plt.plot(angMax["timestamp"]-tStart, movAvg(RadDeg(angMax["MP"])),
         color='gold', linestyle=':')
plt.plot(angMax["timestamp"]-tStart, RadDeg(angMax["DP"]), color='red', alpha=0.3)
plt.plot(angMax["timestamp"]-tStart, movAvg(RadDeg(angMax["DP"])),
         color='red', linestyle=':')
## extended
plt.plot(angMin["timestamp"]-tStart, RadDeg(angMin["PP"]), color='green', alpha=0.3)#, label="$α_{min}$")
plt.plot(angMin["timestamp"]-tStart, movAvg(RadDeg(angMin["PP"])),
         color='green', linestyle=':')#, label="$α_{min}$ (average per minute)")
plt.plot(angMin["timestamp"]-tStart, RadDeg(angMin["MP"]), color='orange', alpha=0.3)
plt.plot(angMin["timestamp"]-tStart, movAvg(RadDeg(angMin["MP"])),
         color='orange', linestyle=':')
plt.plot(angMin["timestamp"]-tStart, RadDeg(angMin["DP"]), color='crimson', alpha=0.3)
plt.plot(angMin["timestamp"]-tStart, movAvg(RadDeg(angMin["DP"])),
         color='crimson', linestyle=':')
plt.tick_params(axis="y")
## legend fake plots
plt.plot(np.nan, np.nan, color='limegreen', label="PP flexed")
plt.plot(np.nan, np.nan, color='green', label="PP extended")
plt.plot(np.nan, np.nan, color='gold', label="MP flexed")
plt.plot(np.nan, np.nan, color='orange', label="MP extended")
plt.plot(np.nan, np.nan, color='red', label="DP flexed")
plt.plot(np.nan, np.nan, color='crimson', label="DP extended")
plt.plot(np.nan, np.nan, color='black', linestyle=':', label="avg/min")

plt.title("extreme position angles")
plt.xlabel("t [s]")
plt.ylabel("α [°]")
plt.legend(loc='upper right')
# fig.legend(bbox_to_anchor=(1,1), bbox_transform=ax1.transAxes)
plt.grid(True)
plt.rc('grid', alpha=0.5, linewidth=0.5)
ExportPDF("extrAngles")
plt.show


### plot DP1 speed & acceleration ###
plt.figure(5)
fig, ax1 = plt.subplots()
ax1.set_xlabel("t [s]")
ax1.set_ylabel(r"$v\ [\frac{m}{s}]$")
ax1.plot(tShift[0:1000], [speedDP1[i] for i in movIdx[0:1000]], color='red', alpha=0.5)
# plt.plot(tShift[0:1000], [speedDP1[i] for i in movIdx[0:1000]], color='red')
ax1.tick_params(axis="y")
ax2 = ax1.twinx()
ax2.set_ylabel(r"$a\ [\frac{m}{s^2}]$")
ax2.plot(tShift[0:1000], [accelDP1[i] for i in movIdx[0:1000]], color='crimson', alpha=0.5)
# plt.plot(tShift[0:1000], [accelDP1[i] for i in movIdx[0:1000]], color='crimson')
ax2.tick_params(axis="y")
fig.subplots_adjust()
## legend fake plots
plt.plot(np.nan, np.nan, color='red', label="v")
plt.plot(np.nan, np.nan, color='crimson', label="a")
plt.title("fingertip speed & acceleration first 10 s")
# plt.xlabel("t [s]")
# plt.ylabel("v [m/s] | a [m/s^2]")
plt.legend()
# plt.grid(True)
# plt.rc('grid', alpha=0.5, linewidth=0.5)
plt.show
