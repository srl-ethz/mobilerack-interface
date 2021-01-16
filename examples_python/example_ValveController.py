from mobilerack_pybind_module import ValveController
from time import sleep

# @todo try to run this code!

valves = [14, 15]
pressure = 300
max_pressure = 500
vc = ValveController("192.168.0.100", valves, max_pressure)

for i in range(len(valves)):
    print(f"index:{i}\tvalve id:{valves[i]}\tpressure:{pressure}")
    vc.setSinglePressure(i, pressure)
    sleep(1)
    vc.setSinglePressure(i, 0)
    sleep(1)
