#include "mobilerack-interface/ValveController.h"
/**
 * @file example_ValveController.cpp
 * @brief This program shows an example usage of the ValveController. Actuates through each valve defined in map
 */

int main() {
    //const int pressure = 500;  //pressure variable value in mBar, can define multiple different ones
    int valve_id;
    std::vector<int> map = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};         //select valves 0, 1, 2, 3, ... 15
    ValveController vc{"192.168.0.100", map, 3500}; //last parameter sets max. pressure in mBar
    

// hand prototype 2 v2
    //hand prototype 2 v2


    //set metacarpal joint angle
    const int angle = 90;


    //translate angle
    const int pressure = angle * 1800 / 90;  // 90 degrees = 2bars


    //linear increase basic
    /*
    const double duration = 5;
    const double timestep = 0.01;
    double p;

    for (double time = 0; time < duration; time += timestep) {
        p = pressure * (time / duration);

        //Pinky flexion
        vc.setSinglePressure(0, 0.98*p);  //middle        
        vc.setSinglePressure(1, 0.8*p);  //bottom       
        vc.setSinglePressure(2, p);  //top        
        
        //ring flexion
        vc.setSinglePressure(3, 0.9 * 0.97*p);  //middle        
        vc.setSinglePressure(4, 0.9 * 0.8*p);  //bottom       
        vc.setSinglePressure(5, p);  //top   
        
        //thumb flexion
        vc.setSinglePressure(12, 0.6*p);  //bottom       
        vc.setSinglePressure(13, 0.7*p);  //top  
        

        //if (SensorReadout > thershold) {
        //    break; }
        srl::sleep(timestep);
    }
    */


    //Peace sign linear increase
    const double duration = 2;
    const double timestep = 0.01;
    double p;

    for (double time = 0; time < duration; time += timestep) {
        p = pressure * (time / duration);

        //Pinky flexion
        vc.setSinglePressure(0, 0.98*p);  //middle        
        vc.setSinglePressure(1, 0.8*p);  //bottom       
        vc.setSinglePressure(2, p);  //top        
        
        //ring flexion
        vc.setSinglePressure(3, 0.9 * 0.97*p);  //middle        
        vc.setSinglePressure(4, 0.9 * 0.8*p);  //bottom       
        vc.setSinglePressure(5, p);  //top   
        
        //thumb flexion
        vc.setSinglePressure(12, 0.6*p);  //bottom       
        vc.setSinglePressure(13, 0.7*p);  //top  
        
        //extensor for index and middle finger
        vc.setSinglePressure(14, 0.75*p);                  

        //if (SensorReadout > thershold) {
        //    break; }
        srl::sleep(timestep);
    }



    srl::sleep(5);


    for (double time = 0; time < duration; time += timestep) {
            p = pressure * (time / duration);

            //Pinky relax
            vc.setSinglePressure(0, pressure-0.98*p);  //middle        
            vc.setSinglePressure(1, pressure-0.8*p);  //bottom       
            vc.setSinglePressure(2, pressure-p);  //top        
            
            //ring relax
            vc.setSinglePressure(3, pressure-0.9*0.97*p);  //middle        
            vc.setSinglePressure(4, pressure-0.9*0.8*p);  //bottom       
            vc.setSinglePressure(5, pressure-p);  //top   
            
            //thumb relax
            vc.setSinglePressure(12, pressure-0.6*p);  //bottom       
            vc.setSinglePressure(13, pressure-0.7*p);  //top  
            
            //extensor for index and middle finger
            vc.setSinglePressure(14, pressure-0.75*p);                  

            srl::sleep(timestep);
        }



/*
    //Pinky flexion
    vc.setSinglePressure(0, 700);  //middle       
    vc.setSinglePressure(1, 700);  //bottom       
    vc.setSinglePressure(2, 700);  //top       
    srl::sleep(1);
    vc.setSinglePressure(0, 1000);  //middle        
    vc.setSinglePressure(1, 1000);  //bottom       
    vc.setSinglePressure(2, 1000);  //top        
    srl::sleep(1);
    vc.setSinglePressure(0, 1650);  //middle        
    vc.setSinglePressure(1, 1200);  //bottom       
    vc.setSinglePressure(2, 1700);  //top        
    srl::sleep(3);

    //pinky relax
    vc.setSinglePressure(0, 0);
    vc.setSinglePressure(1, 0);         
    vc.setSinglePressure(2, 0);         
    srl::sleep(0);



    //Ring flexion
    vc.setSinglePressure(3, 700);  //middle       
    vc.setSinglePressure(4, 700);  //bottom       
    vc.setSinglePressure(5, 700);  //top       
    srl::sleep(1);
    vc.setSinglePressure(3, 1000);  //middle        
    vc.setSinglePressure(4, 1000);  //bottom       
    vc.setSinglePressure(5, 1000);  //top        
    srl::sleep(1);
    vc.setSinglePressure(3, 1600);  //middle        
    vc.setSinglePressure(4, 1100);  //bottom       
    vc.setSinglePressure(5, 1700);  //top        
    srl::sleep(3);

    //Ring relax
    vc.setSinglePressure(3, 0);
    vc.setSinglePressure(4, 0);         
    vc.setSinglePressure(5, 0);         
    srl::sleep(0);




    //Middle flexion
    vc.setSinglePressure(6, 700);  //middle       
    vc.setSinglePressure(7, 700);  //bottom       
    vc.setSinglePressure(8, 700);  //top       
    srl::sleep(1);
    vc.setSinglePressure(6, 1000);  //middle        
    vc.setSinglePressure(7, 1000);  //bottom       
    vc.setSinglePressure(8, 1000);  //top        
    srl::sleep(1);
    vc.setSinglePressure(6, 1600);  //middle        
    vc.setSinglePressure(7, 1100);  //bottom       
    vc.setSinglePressure(8, 1750);  //top        
    srl::sleep(3);

    //Middle relax
    vc.setSinglePressure(6, 0);
    vc.setSinglePressure(7, 0);         
    vc.setSinglePressure(8, 0);         
    srl::sleep(0);

    
    
    //Index flexion
    vc.setSinglePressure(9, 700);  //middle       
    vc.setSinglePressure(10, 700);  //bottom       
    vc.setSinglePressure(11, 700);  //top       
    srl::sleep(1);
    vc.setSinglePressure(9, 1000);  //middle        
    vc.setSinglePressure(10, 1000);  //bottom       
    vc.setSinglePressure(11, 1000);  //top        
    srl::sleep(1);
    vc.setSinglePressure(9, 1650);  //middle        
    vc.setSinglePressure(10, 1100);  //bottom       
    vc.setSinglePressure(11, 1700);  //top        
    srl::sleep(3);

    //Index relax
    vc.setSinglePressure(9, 0);
    vc.setSinglePressure(10, 0);         
    vc.setSinglePressure(11, 0);         
    srl::sleep(0);



    //Thumb flexion      
    vc.setSinglePressure(12, 700);  //bottom       
    vc.setSinglePressure(13, 700);  //top       
    srl::sleep(1);      
    vc.setSinglePressure(12, 1000);  //bottom       
    vc.setSinglePressure(13, 1000);  //top        
    srl::sleep(1);       
    vc.setSinglePressure(12, 1300);  //bottom       
    vc.setSinglePressure(13, 1600);  //top        
    srl::sleep(3);

    //Thumb relax
    vc.setSinglePressure(12, 0);         
    vc.setSinglePressure(13, 0);         
    srl::sleep(0);


    //Long Fingers extensor      
    vc.setSinglePressure(14, 500);                 
    srl::sleep(1);
    vc.setSinglePressure(14, 1000);                 
    srl::sleep(2);  
    vc.setSinglePressure(14, 1400);                 
    srl::sleep(2); 


    //Thumb extensor      
    vc.setSinglePressure(15, 500);                 
    srl::sleep(1);
    vc.setSinglePressure(15, 1000);                 
    srl::sleep(2);  
    vc.setSinglePressure(15, 1400);                 
    srl::sleep(2); 

    //Extensors relax
    vc.setSinglePressure(14, 0);  
    vc.setSinglePressure(15, 0);                 
    srl::sleep(2); 




//Full Flexion of all fingers

    //Thumb flexion      
    vc.setSinglePressure(12, 700);  //bottom       
    vc.setSinglePressure(13, 700);  //top       
    srl::sleep(1);      
    vc.setSinglePressure(12, 1000);  //bottom       
    vc.setSinglePressure(13, 1000);  //top        
    srl::sleep(1);       
    vc.setSinglePressure(12, 1300);  //bottom       
    vc.setSinglePressure(13, 1600);  //top        
    srl::sleep(1);

    //Pinky flexion
    vc.setSinglePressure(0, 700);  //middle       
    vc.setSinglePressure(1, 700);  //bottom       
    vc.setSinglePressure(2, 700);  //top       
    srl::sleep(1);
    vc.setSinglePressure(0, 1000);  //middle        
    vc.setSinglePressure(1, 1000);  //bottom       
    vc.setSinglePressure(2, 1000);  //top        
    srl::sleep(1);
    vc.setSinglePressure(0, 1700);  //middle        
    vc.setSinglePressure(1, 1200);  //bottom       
    vc.setSinglePressure(2, 1800);  //top        
    srl::sleep(1);

    //Ring flexion
    vc.setSinglePressure(3, 700);  //middle       
    vc.setSinglePressure(4, 700);  //bottom       
    vc.setSinglePressure(5, 700);  //top       
    srl::sleep(1);
    vc.setSinglePressure(3, 1000);  //middle        
    vc.setSinglePressure(4, 1000);  //bottom       
    vc.setSinglePressure(5, 1000);  //top        
    srl::sleep(1);
    vc.setSinglePressure(3, 1650);  //middle        
    vc.setSinglePressure(4, 1150);  //bottom       
    vc.setSinglePressure(5, 1750);  //top        
    srl::sleep(1);

    //Middle flexion
    vc.setSinglePressure(6, 700);  //middle       
    vc.setSinglePressure(7, 700);  //bottom       
    vc.setSinglePressure(8, 700);  //top       
    srl::sleep(1);
    vc.setSinglePressure(6, 1000);  //middle        
    vc.setSinglePressure(7, 1000);  //bottom       
    vc.setSinglePressure(8, 1000);  //top        
    srl::sleep(1);
    vc.setSinglePressure(6, 1550);  //middle        
    vc.setSinglePressure(7, 1100);  //bottom       
    vc.setSinglePressure(8, 1700);  //top        
    srl::sleep(1);

    //Index flexion
    vc.setSinglePressure(9, 700);  //middle       
    vc.setSinglePressure(10, 700);  //bottom       
    vc.setSinglePressure(11, 700);  //top       
    srl::sleep(1);
    vc.setSinglePressure(9, 1000);  //middle        
    vc.setSinglePressure(10, 1000);  //bottom       
    vc.setSinglePressure(11, 1000);  //top        
    srl::sleep(1);
    vc.setSinglePressure(9, 1300);  //middle        
    vc.setSinglePressure(10, 1000);  //bottom       
    vc.setSinglePressure(11, 1400);  //top        
    srl::sleep(1);

    
    



    srl::sleep(10);

*/

    //End / Relax all
    vc.setSinglePressure(0, 0);
    vc.setSinglePressure(1, 0);         
    vc.setSinglePressure(2, 0);
    vc.setSinglePressure(3, 0);
    vc.setSinglePressure(4, 0);         
    vc.setSinglePressure(5, 0);
    vc.setSinglePressure(6, 0);
    vc.setSinglePressure(7, 0);         
    vc.setSinglePressure(8, 0);
    vc.setSinglePressure(9, 0);
    vc.setSinglePressure(10, 0);         
    vc.setSinglePressure(11, 0);
    vc.setSinglePressure(12, 0);
    vc.setSinglePressure(13, 0);         
    vc.setSinglePressure(14, 0);
    vc.setSinglePressure(15, 0);


// hand prototype 2 v1 end

}
