# from mobilerack_pybind_module import ValveController, QualisysClient
from time import sleep
import time
import datetime
from interval_timer import IntervalTimer
import numpy as np
import pandas as pd
import csv
from pathlib import Path

path = str(Path(__file__).parent)

print("Which file should be analized?")
# fileName=input()
fileName="test4"




### Inspired by ChatGPT ###

# # Definieren Sie eine Funktion zum Laden der Daten aus der Textdatei
# def load_data(file_path):
#     dataList_arrays = []
#     with open(file_path, 'r') as file:
#         lines = file.readlines()
#         data_lines=lines[1:]
#         for line in data_lines:
#             # Entfernen Sie Zeilenumbrüche und trennen Sie die Liste in einzelne Datenpunkte auf
#             data = line.strip()[1:-1].split('], [')
#             # Konvertieren Sie die Zeichenketten in Listen um
#             cleaned_data = []
#             for entry in data:
#                 values = entry.replace('[', '').replace(']', '').split(', ')
#                 values = [float(value) for value in values if value.strip() != '']
#                 cleaned_data.append(values)
#             # data = [list(map(float, d.split(', '))) for d in data]
#             dataList_arrays.append(cleaned_data)
#     return dataList_arrays

# # Dateipfad zur Textdatei
# file_path = 'ExportData/' + fileName + '.txt'

# # Laden der Daten aus der Textdatei
# imported_dataList_arrays = load_data(file_path)

# # Überprüfen der importierten Daten
# for dataList in imported_dataList_arrays:
#     print(dataList)




with open('ExportData/'+fileName+'.txt') as file:
    header = file.readlines()[0]
with open('ExportData/'+fileName+'.txt') as file:
    data = file.readlines()[1:]
    
i = 0
for line in data:
    data[i] = line.strip()[1:-1].split('], [')
    i += 1
    for entry in data:
        