from mobilerack_pybind_module import ValveController, QualisysClient
from time import sleep

valves = [14, 15]
max_pressure = 500
pressure = 300

vc = ValveController("192.168.0.100", valves, max_pressure)
qc = QualisysClient("192.168.0.101", 0)

sleep(1)  # hacky way to wait until data from qtm is received
_, timestamp = qc.getData()
vc.syncTimeStamp(timestamp//1000)  # sync the time to be that of QTM

for i in range(len(valves)):
    print(f"index:{i}\tvalve id:{valves[i]}\tpressure:{pressure}")
    vc.setSinglePressure(i, pressure)
    sleep(1)
    vc.setSinglePressure(i, 0)
    sleep(1)
