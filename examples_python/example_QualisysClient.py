from mobilerack_pybind_module import QualisysClient
from time import sleep

qc = QualisysClient("192.168.0.101", 2)

for i in range(10):
    frames, timestamp = qc.getData()
    print("-"*10)
    print(f"number of frames:{len(frames)}\ttimestamp:{timestamp}\ndata:{frames}")
    sleep(0.5)