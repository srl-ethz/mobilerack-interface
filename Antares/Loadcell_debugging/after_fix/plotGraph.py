#! /usr/bin/env python3
import csv
import matplotlib.pyplot as plt
import sys
from os import path

"""
This is a Python3 script to plot the logged data.
Use with filename after command, such as '$./plotGraph.py log.csv'
Assumes the first column is the x values (usually the time) and the subsequent columns are y values. 
"""
x_label = "" # label for leftmost column (usually the time)
x_data = [] # data for the leftmost column
label_array = [] # labels on first row (excluding first element)
data_array = []

try:
    filename = sys.argv[1]
except IndexError:
    print("Error: Put in filename as argument")
    exit()
basename = path.splitext(path.basename(filename))[0] # filename but without the extension, to be used as graph title

with open(filename) as csvfile:
    reader = csv.reader(csvfile)

    is_first_row = True
    for row in reader:
        if is_first_row:
            x_label = row[0]
            for i in range(1, len(row)):
                label_array.append(row[i])
                data_array.append([])
            print(f"plotting x axis: \"{x_label}\", and values: {label_array}")
            is_first_row = False
            continue
        if len(row) < len(label_array) + 1:
            print(f"WARNING: ignoring a row because it contains only {len(row)} elements.")
            continue
        x_data.append(float(row[0]))
        for i in range(len(label_array)):
            data_array[i].append(float(row[i+1]))

for i in range(len(label_array)):
    plt.plot(x_data, data_array[i], ".-", label=label_array[i])
plt.xlabel(x_label)
plt.legend()
plt.title(basename)
plt.show()

