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
    angle = 90


    #translate angle
    pressure = angle * 1600 / 90;  # 90 degrees = 2bars
    break_pressure = -1


    #linear increase
    duration = 4
    timestep = 0.01
    

    for time in np.arange(0, duration, timestep):
    #for (double time = 0; time < duration; time += timestep) {
        p = pressure * (time / duration)
        #print(int(p))

        #Pinky flexion
        vc.setSinglePressure(0, int(0.98*p) );    #middle        
        vc.setSinglePressure(1, int(0.8*p) );     #bottom       
        vc.setSinglePressure(2, int(p) );         #top        
        
        #ring flexion
        vc.setSinglePressure(3, int(0.9 * 0.97*p) );  #middle        
        vc.setSinglePressure(4, int(0.9 * 0.8*p) );   #bottom       
        vc.setSinglePressure(5, int(p) );             #top   

        #middle flexion
        vc.setSinglePressure(6, int(0.9 * 0.97*p) );  #middle        
        vc.setSinglePressure(7, int(0.9 * 0.8*p) );   #bottom       
        vc.setSinglePressure(8, int(p) );             #top  

        #index flexion
        vc.setSinglePressure(9, int(0.9 * 0.97*p) );  #middle        
        vc.setSinglePressure(10, int(0.9 * 0.8*p) );   #bottom       
        vc.setSinglePressure(11, int(p) );             #top   
        
        #thumb flexion
        vc.setSinglePressure(12, int(0.6*p) );  #bottom       
        vc.setSinglePressure(13, int(0.7*p) );  #top  
        
        #extensor for index
        vc.setSinglePressure(14, int(0.7*p) );                  


        line = ser.readline()           #serial readout command
        try: 
            sensor = float(line)             #convert it to float
            print(sensor)                    #print the float
        except ValueError:                  #avoid random error breakouts
            continue

        if (sensor > 40):
            break_pressure = p
            break; 
        
        sleep(timestep)

        
    



    sleep(5)

    for time in np.arange(0, duration, timestep):
    #for (double time = 0; time < duration; time += timestep) {
        if break_pressure != -1:
            pressure = break_pressure

        p = pressure * (time / duration)

        #Pinky relax
        vc.setSinglePressure(0, int(pressure-0.98*p) );   #middle        
        vc.setSinglePressure(1, int(pressure-0.8*p) );    #bottom       
        vc.setSinglePressure(2, int(pressure-p) );        #top        
        
        #ring relax
        vc.setSinglePressure(3, int(pressure-0.9*0.97*p) );   #middle        
        vc.setSinglePressure(4, int(pressure-0.9*0.8*p) );    #bottom       
        vc.setSinglePressure(5, int(pressure-p) );            #top  

        #middle relax
        vc.setSinglePressure(6, int(pressure-0.9*0.97*p) );   #middle        
        vc.setSinglePressure(7, int(pressure-0.9*0.8*p) );    #bottom       
        vc.setSinglePressure(8, int(pressure-p) );            #top  

        #index relax
        vc.setSinglePressure(9, int(pressure-0.9*0.97*p) );   #middle        
        vc.setSinglePressure(10, int(pressure-0.9*0.8*p) );    #bottom       
        vc.setSinglePressure(11, int(pressure-p) );            #top  
        
        #thumb relax
        vc.setSinglePressure(12, int(pressure-0.6*p) );  #bottom       
        vc.setSinglePressure(13, int(pressure-0.7*p) );  #top  
        
        #extensor for index and middle finger
        vc.setSinglePressure(14, int(pressure-0.7*p) );                  

        sleep(timestep)
        
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