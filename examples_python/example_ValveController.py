from mobilerack_pybind_module import ValveController
from time import sleep
import serial
import struct
import numpy as np
# @todo try to run this code!

valves = [14, 15]
pressure = 300
max_pressure = 500
vc = ValveController("192.168.0.100", valves, max_pressure)
ser = serial.Serial('/dev/ttyACM0')
line = ser.readline()

#for i in range(len(valves)):
#    print(f"index:{i}\tvalve id:{valves[i]}\tpressure:{pressure}")
#    vc.setSinglePressure(i, pressure)
#    vc.setSinglePressure(i, 0)


while True:                         #make it run continously
    line = ser.readline()           #serial readout command
    try: 
        sensor = float(line)             #convert it to float
        print(sensor)                    #print the float
    except ValueError:                  #avoid random error breakouts
        continue
    


#hand prototype 2 v2


    #set metacarpal joint angle
    angle = 90;


    #translate angle
    pressure = angle * 1600 / 90;  # 90 degrees = 2bars


    #linear increase
    duration = 4;
    timestep = 0.01;
    

    for time in np.arange(0, duration, timestep):
    #for (double time = 0; time < duration; time += timestep) {
        p = pressure * (time / duration);

        #Pinky flexion
        vc.setSinglePressure(0, 0.98*p);    #middle        
        vc.setSinglePressure(1, 0.8*p);     #bottom       
        vc.setSinglePressure(2, p);         #top        
        
        #ring flexion
        vc.setSinglePressure(3, 0.9 * 0.97*p);  #middle        
        vc.setSinglePressure(4, 0.9 * 0.8*p);   #bottom       
        vc.setSinglePressure(5, p);             #top   
        
        #thumb flexion
        vc.setSinglePressure(12, 0.6*p);  #bottom       
        vc.setSinglePressure(13, 0.7*p);  #top  
        
        #extensor for index and middle finger
        vc.setSinglePressure(14, 0.75*p);                  

        if (SensorReadout > thershold) {
            break; }
        
        sleep(timestep);
    }



    sleep(5);


    for (double time = 0; time < duration; time += timestep) {
            p = pressure * (time / duration);

            #Pinky relax
            vc.setSinglePressure(0, pressure-0.98*p);   #middle        
            vc.setSinglePressure(1, pressure-0.8*p);    #bottom       
            vc.setSinglePressure(2, pressure-p);        #top        
            
            #ring relax
            vc.setSinglePressure(3, pressure-0.9*0.97*p);   #middle        
            vc.setSinglePressure(4, pressure-0.9*0.8*p);    #bottom       
            vc.setSinglePressure(5, pressure-p);            #top   
            
            #thumb relax
            vc.setSinglePressure(12, pressure-0.6*p);  #bottom       
            vc.setSinglePressure(13, pressure-0.7*p);  #top  
            
            #extensor for index and middle finger
            vc.setSinglePressure(14, pressure-0.75*p);                  

            sleep(timestep);
        }