#! /usr/local/bin/python3
import csv
import matplotlib.pyplot as plt
import sys

"""
This is a Python3 script to plot the logged data.")
Use with filename after command, such as '$python3 plotGraph.py log.csv'
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

with open(filename) as csvfile:
    reader = csv.reader(csvfile)

    is_first_row = True
    for row in reader:
        if is_first_row:
            x_label = row[0]
            for i in range(1, len(row)):
                label_array.append(row[i])
                data_array.append([])
            print(f"plotting x axis:\t{x_label} and values:\t{label_array}")
            is_first_row = False
            continue
        x_data.append(float(row[0]))
        for i in range(len(label_array)):
            data_array[i].append(float(row[i+1]))

for i in range(len(label_array)):
    plt.plot(x_data, data_array[i], ".-", label=label_array[i])
plt.xlabel(x_label)
plt.legend()
plt.show()

