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
cameras = [i for i in range(10)]
qc = QualisysClient(6, cameras, "3D")

sleep(2)  # hacky way to wait until data is received from QTM
for i in range(100):
    frames, timestamp = qc.getData3D()
    print(f"number of frames:{len(frames)}\ttimestamp:{timestamp}\ndata:{frames}")
    # if len(cameras) > 0:
    #     image = qc.getImage(0)
    #     cv2.imshow("image", image)
    #     cv2.waitKey(1)
    sleep(0.1)

cv2.destroyAllWindows()