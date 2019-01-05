import csv
import numpy as np
import math
import matplotlib.pyplot as plt

def read_csv(csvName):
    py_array = []
    with open(csvName) as csvFile:
        reader = csv.reader(csvFile)
        for row in reader:
            py_array.append([float(element) for element in row])
    return np.array(py_array)

segments = 2;
chambers = 3;
print("make sure that the number of chambers and segments are correct.")
print("Number of segments\t", segments)
print("Chambers per segment\t", chambers)

f= read_csv("characterization_f.csv")[:,1:]
q=read_csv("characterization_q.csv")[:,1:]
dq=read_csv("characterization_dq.csv")[:,1:]
p=read_csv("characterization_p.csv")[:,1:]

if chambers == 3:
    A_p2f = np.matrix([[1, -1/2, -1/2],
                       [0, math.sqrt(3)/2, -math.sqrt(3)/2]])
elif chambers == 4:
    A_p2f = np.matrix([[1, 0, -1, 0],
                      [0, 1, 0, -1]])

history_size = f.shape[1]
print("history size\t", history_size)
history_matrix = np.zeros([segments*2+1, history_size*2*segments])
for i in range(history_size):
    history_matrix[0:2*segments, 2*segments*i:2*segments*i+2*segments] = -np.diag(q[:,i])
    history_matrix[2*segments: 2*segments+1, 2*segments*i:2*segments*i+2*segments] = -np.transpose(dq[:,i])
f_asRow = np.zeros([2*segments*history_size, 1])
for i in range(history_size):
    f_asRow[2*segments*i:2*segments*i+2*segments, 0] = np.transpose(f[:,i])
# print("history matrix", history_matrix)
# print("pinv of history matrix", np.linalg.pinv(history_matrix))
# print("f_asRow", f_asRow)
print("If there are nan values, there may be nan values in the logged data. Edit to remove them.")
result = np.matmul(np.transpose(np.linalg.pinv(history_matrix)),  f_asRow)

print("[k(0), k(1), k(2), ... , d]")
print(result[:,0])

calculated_f_asRow = np.matmul(np.transpose(result), history_matrix)

calculated_f = np.zeros([2*segments, history_size])
for i in range(history_size):
    calculated_f[:,i] = np.transpose(calculated_f_asRow[0, 2*segments*i:2*segments*i+2*segments])

for i in range(2*segments):
    plt.plot(f[i,:], label="measured values for segment "+str(i//2)+" direction "+str(i%2))
    plt.plot(calculated_f[i,:], label = "calculated values for segment "+str(i//2)+" direction "+str(i%2))

plt.title("comparison of characterization result & reality")
plt.legend()
plt.show()
