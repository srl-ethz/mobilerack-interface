#! /usr/local/bin/python3
import csv
import numpy as np
import matplotlib.pyplot as plt
import sys

values = dict()

print("This is a Python3 script to plot the logged data.")
print("Use with filename after command, such as '$python3 plotGraph.py log.csv'")
print("You can selectively plot data by including the labels as arguments, like '$python3 plotGraph.py log.csv q_ref[0] q_meas[0]'")

try:
    filename = sys.argv[1]
except IndexError:
    print("Error: Put in filename as argument")
    exit()

# select which data to plot
selectedData = []
if len(sys.argv) > 2:
    selectedData = sys.argv[2:]

with open(filename) as csvfile:
    reader = csv.DictReader(csvfile)
    firstRow = True
    for row in reader:
        # first, if there were arguments designating which data to plot, remove the columns supposed to be removed
        newDict = dict(row)
        if len(selectedData)>1:
            for key in row:
                if not (key.replace(" ", "") in selectedData or key == 'time(millis)'):
                    del newDict[key]

        if firstRow:
            # create list of dictionarys that store data
            if not 'time(millis)' in newDict:
                print("no column titled 'time(millis)'. The CSV is not formatted correctly.")
                exit()
            for key in newDict:
                values[key] = np.array([])
            firstRow = False
        else:
            for key in newDict:
                values[key] = np.append(values[key], newDict[key])


for key in values:
    print("column found :",key)
    if key != "time(millis)":
        plt.plot(values["time(millis)"], values[key], ".-", label=key)
plt.xlabel('time(millis)')
plt.legend()
plt.show()

