from mobilerack_pybind_module import ValveController, QualisysClient
from time import sleep
import time
import numpy as np


cameras = [0,1,2,3,4,5,6,7]
valves = [0,1,2]
max_pressure = 400

# Define all the K pressures you want to try out, shape [K, V], with V valves active.
pressures = [
    [350, 0, 0],
    [0, 350, 0],
    #[0, 0, 350]
]

timesteps = 200
number_of_markers = 7

vc = ValveController("192.168.0.100", valves, max_pressure)
qc = QualisysClient(number_of_markers, cameras, "3D")

sleep(1)  # hacky way to wait until data from qtm is received
_, timestamp = qc.getData3D()
vc.syncTimeStamp(timestamp//1000)  # sync the time to be that of QTM

captured_markers = []
for k in range(len(pressures)):
    start_time = time.time()

    frame_list = []
    curr_stamp = timestamp
    for t in range(timesteps):
        while timestamp == curr_stamp:
            frames, timestamp = qc.getData3D()
        frame_list += [np.array(frames)]
        curr_stamp = timestamp

        for i in range(len(valves)):
            vc.setSinglePressure(i, pressures[k][i])
                
    total_time = time.time()-start_time
    print(f"Ran for {total_time:.3f}")
    print(f"Timing accuracy: {100*(total_time-timesteps/100)/timesteps/100}")

    captured_markers.append(np.array(frame_list))     # frame_list has shape [T, N, 3] for N motion markers


    for i in range(len(valves)):
        vc.setSinglePressure(i, 0)
    # Reset
    sleep(1)

captured_markers = np.array(captured_markers)
captured_info = {'p': np.array(pressures), 'data': captured_markers}
np.save("captured_data.npy", captured_info)

