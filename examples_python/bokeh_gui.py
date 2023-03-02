from mobilerack_pybind_module import ValveController
from time import sleep
import threading
import numpy as np

from bokeh.io import curdoc
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, Slider, TextInput
from bokeh.plotting import figure

"""
GUI with sliders for controlling the pressure of each valve.

install bokeh with
`pip install bokeh`
run with
`bokeh serve bokeh_gui.py`
and navigate to the URL with your browser
"""

valves = [0, 1, 2, 3]
max_pressure = 500
vc = ValveController("192.168.0.100", valves, max_pressure)


num_valves = len(valves)
target_pressures = np.zeros(num_valves)
targets_updated = False

def update():
    global targets_updated
    global target_pressures
    while True:
        if targets_updated:
            for valve_idx in range(num_valves):
                vc.setSinglePressure(valve_idx, int(target_pressures[valve_idx]))
            targets_updated = False
        sleep(0.01)

target_update_thread = threading.Thread(target=update)
target_update_thread.start()

# Create sliders 
pressure_ctrl_sliders = [Slider(title=f"Valve {valves[valve_idx]}", value=0, start=0, end=max_pressure, step=10) for valve_idx in range(num_valves)]

def update_data(attrname, old, new):
    global targets_updated
    global target_pressures
    # Get the current slider values
    target_pressures = np.array([slider.value for slider in pressure_ctrl_sliders])
    targets_updated = True

for slider in pressure_ctrl_sliders:
    slider.on_change('value', update_data)

# Set up layouts and add to document
inputs = column(list(pressure_ctrl_sliders), width=320, height=1000)

curdoc().add_root(row(inputs, width=800))
curdoc().title = "Valve Control"