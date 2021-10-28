from mobilerack_pybind_module import ValveController
from time import sleep
import serial
import struct
import numpy as np
# @todo try to run this code!

valves = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14, 15]
pressure = 300
max_pressure = 1600
vc = ValveController("192.168.0.100", valves, max_pressure)
ser = serial.Serial('/dev/ttyACM0')
line = ser.readline()

#for i in range(len(valves)):
#    print(f"index:{i}\tvalve id:{valves[i]}\tpressure:{pressure}")
#    vc.setSinglePressure(i, pressure)
#    vc.setSinglePressure(i, 0)

try:
#while True:                         #make it run continously
#    line = ser.readline()           #serial readout command
#    try: 
#        sensor = float(line)             #convert it to float
#        print(sensor)                    #print the float
#    except ValueError:                  #avoid random error breakouts
#        continue
    


#hand prototype 2 v2


    #set metacarpal joint angle


    #translate angle
    pressure = 1800
    p_thumb = 1200;
    break_pressure = -1


    #linear increase
    duration = 7
    timestep = 0.01






    for time in np.arange(0, duration, timestep):
    #for (double time = 0; time < duration; time += timestep) {
        p = pressure * (time / duration)
        pt = p_thumb * (time / duration);
        #print(int(p))

        line = ser.readline()           #serial readout command
        try: 
            #sensor = float(line)             #convert it to float
            sensor = list(map(float, str(line.decode("utf-8")).split(", ")))   #sensor list 0,1,2,3,4
            print(sensor)                    #print the float
        except ValueError:                  #avoid random error breakouts
            continue

        #if (sensor > 40):
        #    break_pressure = p
        #    break; 


        if (sensor[1] < 40):
            #Pinky & ring flexion - sensor 0,1
            vc.setSinglePressure(0, int(0.95*p) );    #middle        
            vc.setSinglePressure(1, int(0.85*p) );     #bottom       
            vc.setSinglePressure(2, int(p) );         #top   

        if (sensor[2] < 40):
            #middle flexion - sensor 2
            vc.setSinglePressure(3, int(0.95*0.95*p) );  #middle        
            vc.setSinglePressure(4, int(0.95*0.85*p) );   #bottom       
            vc.setSinglePressure(5, int(0.95*p) );             #top   

        #if (sensor[3] < 40):
        #index flexion - no working sensor
        vc.setSinglePressure(6, int(0.9 * 0.95*p) );  #middle        
        vc.setSinglePressure(7, int(0.9 * 0.85*p) );   #bottom       
        vc.setSinglePressure(8, int(0.9*p) );             #top  

        if (sensor[3] < 40):
            #thumb flexion
            vc.setSinglePressure(12, int(0.8*pt));  #bottom thumb flexor      
            vc.setSinglePressure(11, int(0.9*pt));  #top thumb flexor
            vc.setSinglePressure(10, int(0.9*pt));  #opponens pollicis
            vc.setSinglePressure(9, int(0.7*pt));  #abductor pollicis brevis   
        
        if (sensor[4] > 60):
            break_pressure = p
            break; 
        
        
        sleep(timestep)

        
        
        
    sleep(10)




        
finally:
    #End / Relax all
    vc.setSinglePressure(0, 0)
    vc.setSinglePressure(1, 0)         
    vc.setSinglePressure(2, 0)
    vc.setSinglePressure(3, 0)
    vc.setSinglePressure(4, 0)         
    vc.setSinglePressure(5, 0)
    vc.setSinglePressure(6, 0)
    vc.setSinglePressure(7, 0)         
    vc.setSinglePressure(8, 0)
    vc.setSinglePressure(9, 0)
    vc.setSinglePressure(10, 0)       
    vc.setSinglePressure(11, 0)
    vc.setSinglePressure(12, 0)
    vc.setSinglePressure(13, 0)       
    vc.setSinglePressure(14, 0)
    vc.setSinglePressure(15, 0)