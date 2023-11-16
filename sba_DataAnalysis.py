from mobilerack_pybind_module import ValveController, QualisysClient
from time import sleep
import time
import datetime
from interval_timer import IntervalTimer
import numpy as np
import pandas as pd
import csv

file = open('ExportData/captured_data.txt', 'r')
data = file.read
print(data)

data = np.loadtxt('ExportData/captured_data.txt', skiprows=1)
print(data[0])
print(data[1])