from mobilerack_pybind_module import QualisysClient
from time import sleep
import time
import matplotlib.pyplot as plt
import numpy as np


cameras = [0,1,2,3,4,5,6,7]

timesteps = 10000
number_of_markers = 10

qc = QualisysClient(number_of_markers, cameras, "3DNoLabels")

sleep(1)  # hacky way to wait until data from qtm is received
_, timestamp = qc.getData3D()

captured_markers = []
start_time = time.time()

frame_list = []
curr_stamp = timestamp
for t in range(timesteps):
    frames, timestamp = qc.getData3D()
    for i,frame in enumerate(frames):
        print(f"Frame {i}: {frame}")
    frame_list += [np.array(frames)]
    curr_stamp = timestamp

total_time = time.time()-start_time
print(f"Ran for {total_time:.3f}")

captured_markers.append(np.array(frame_list))     # frame_list has shape [T, N, 3] for N motion markers


sleep(1)
captured_markers = np.array(captured_markers)
print("captured markers shape ", captured_markers.shape)
# plt.plot(captured_markers)

