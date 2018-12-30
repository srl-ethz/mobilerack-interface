import csv
import numpy as np
import matplotlib.pyplot as plt

def read_csv(csvName):
    py_array = []
    with open(csvName) as csvFile:
        reader = csv.reader(csvFile)
        for row in reader:
            py_array.append([float(element) for element in row])
    return np.array(py_array)

f= read_csv("characterization_f.csv")
history=read_csv("characterization_history.csv")
print("shape of f\t",f.shape)
print("shape of history\t",history.shape)

# sanitize results
for i in range(f.shape[0]):
    pass

result = np.matmul( np.transpose(np.linalg.pinv(history)) ,  f)
print(result)

calculated = np.transpose( np.matmul(np.transpose(result), history))
plt.plot(calculated, label="calculated")
plt.plot(f, label = "measured")
plt.title("comparison of characterization result & reality")
plt.legend()
plt.show()
