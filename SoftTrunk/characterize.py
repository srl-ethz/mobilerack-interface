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

a = 10
b= 190
bound = True
segments = 2;
f= read_csv("characterization_f.csv")
history=read_csv("characterization_history.csv")
if bound:
    f = f[a:b,:]
    history = history[:,a:b]
print("shape of f\t",f.shape)
print("shape of history\t",history.shape)

# sanitize results
for i in range(f.shape[0]):
    pass

result = np.matmul( np.transpose(np.linalg.pinv(history)) ,  f)
print(result)

calculated = np.transpose( np.matmul(np.transpose(result), history))

for i in range(segments):
    data_measured = []
    data_calculated = []
    for j in range(calculated.shape[0]//segments):
        data_calculated.append(calculated[j*segments + i])
        data_measured.append(f[j*segments + i])
    plt.plot(np.array(data_calculated), label="calculated values for segment "+str(i))
    plt.plot(np.array(data_measured), label = "measured values for segment "+str(i))


plt.title("comparison of characterization result & reality")
plt.legend()
plt.show()
