from mobilerack_pybind_module import QualisysClient
import cv2
from time import sleep

"""
Stream frames & images from Qualisys Track Manager.
Images are not streamed in playback mode. (at least, by default)

May need extra package in order to display images:
```bash
pip3 install opencv-python
```
"""
stream_image = True
qc = QualisysClient("192.168.0.101", 2, stream_image)

sleep(2)  # hacky way to wait until data is received from QTM
for i in range(100):
    frames, timestamp = qc.getData()
    print("-"*10)
    print(f"number of frames:{len(frames)}\ttimestamp:{timestamp}\ndata:{frames}")
    if stream_image:
        image1, image2 = qc.getImage()
        cv2.imshow("image1", image1)
        cv2.imshow("image2", image2)
        cv2.waitKey(1)
    sleep(0.1)

cv2.destroyAllWindows()