from mobilerack_pybind_module import QualisysClient
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
cameras = [9]
qc = QualisysClient(2, cameras, "6D")

sleep(2)  # hacky way to wait until data is received from QTM
for i in range(100):
    frames, timestamp = qc.getData6D()
    print("-"*10)
    print(f"number of frames:{len(frames)}\ttimestamp:{timestamp}\ndata:{frames}")
    if len(cameras) > 0:
        image = qc.getImage(0)
        cv2.imshow("image", image)
        cv2.waitKey(1)
    sleep(0.1)

cv2.destroyAllWindows()