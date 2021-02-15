from mobilerack_pybind_module import QualisysClient
import cv2
from time import sleep

"""
Stream frames & images from Qualisys Track Manager. (images are not streamed in playback mode?)
May need extra package in order to display images:
```bash
pip3 install opencv-python
```
"""
stream_image = False
qc = QualisysClient("192.168.0.101", 2, stream_image)

sleep(2)
for i in range(10):
    frames, timestamp = qc.getData()
    print("-"*10)
    print(f"number of frames:{len(frames)}\ttimestamp:{timestamp}\ndata:{frames}")
    if stream_image:
        image = qc.getImage()
        cv2.imshow("image", image)
    sleep(0.5)

cv2.destroyAllWindows()