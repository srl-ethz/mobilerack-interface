from mobilerack import QualisysClient
import cv2
from time import sleep

"""
Stream frames & images from Qualisys Track Manager.
Images are not streamed in playback mode.

May need extra package in order to display images:
```bash
pip3 install opencv-python
```
"""
cameras = [i for i in range(8)]
qc = QualisysClient(1, cameras, "6D")

sleep(2)  # hacky way to wait until data is received from QTM
for i in range(10000):
    frames, timestamp = qc.getData6D()
    print("-"*10)
    print(f"number of frames:{len(frames)}\ttimestamp:{timestamp}\ndata:{frames}")
    print("frames: ")
    print(frames)
