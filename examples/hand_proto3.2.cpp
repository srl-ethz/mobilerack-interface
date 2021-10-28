#include "mobilerack-interface/ValveController.h"
/**
 * @file example_ValveController.cpp
 * @brief This program shows an example usage of the ValveController. Actuates through each valve defined in map
 */

int main() {
    //const int pressure = 500;  //pressure variable value in mBar, can define multiple different ones
    int valve_id;
    std::vector<int> map = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};         //select valves 0, 1, 2, 3, ... 15
    ValveController vc{"192.168.0.100", map, 2000}; //last parameter sets max. pressure in mBar
    

// hand prototype 3 v1
// grasping experiments

    //set metacarpal joint angle
    //const int angle = 90;
    //translate angle
    //const int pressure = angle * 1000 / 90;  // 90 degrees = 2bars

    double duration = 10;
    double timestep = 0.01;
    double p;
    double pt;

    //set lead pressure in mbar
    int pressure = 1800; //1800 easy
    int p_thumb = 1200; //1800 easy




/*
////////////////////////////////////////////////////////////////
//basic grasp flex all fingers together

    pressure = 1800; 
    p_thumb = 1200;


    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring flexion
        vc.setSinglePressure(0, 0.95*p);  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*p);  //bottom pinky and ring        
        vc.setSinglePressure(2, p);  //top pinky and ring         
        
        //middle flexion
        vc.setSinglePressure(3, 0.95*p);  //middle middle finger       
        vc.setSinglePressure(4, 0.84*p);  //bottom middle finger      
        vc.setSinglePressure(5, p);  //top middle finger  

        //index flexion
        vc.setSinglePressure(6, 0.95*p);  //middle index       
        vc.setSinglePressure(7, 0.84*p);  //bottom index      
        vc.setSinglePressure(8, p);  //top index  

        //thumb flexion
        vc.setSinglePressure(12, 0.8*pt);  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*pt);  //top thumb flexor
        vc.setSinglePressure(10, 1*pt);  //opponens pollicis
        //vc.setSinglePressure(9, 0.7*pt);  //abductor pollicis brevis               

        //std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(10);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring relax
        vc.setSinglePressure(0, 0.95*(pressure-p) );  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*(pressure-p) );  //bottom pinky and ring        
        vc.setSinglePressure(2, pressure-p);  //top pinky and ring         
        
        //middle relax
        vc.setSinglePressure(3, 0.95*(pressure-p) );  //middle middle finger       
        vc.setSinglePressure(4, 0.84*(pressure-p) );  //bottom middle finger      
        vc.setSinglePressure(5, pressure-p);  //top middle finger  

        //index relax
        vc.setSinglePressure(6, 0.95*(pressure-p) );  //middle index       
        vc.setSinglePressure(7, 0.84*(pressure-p) );  //bottom index      
        vc.setSinglePressure(8, pressure-p);  //top index  

        //thumb relax
        vc.setSinglePressure(12, 0.8*(p_thumb-pt) );  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*(p_thumb-pt) );  //top thumb flexor
        vc.setSinglePressure(10, 1*(p_thumb-pt) );  //opponens pollicis
        //vc.setSinglePressure(9, 0.7*(p_thumb-pt) );  //abductor pollicis brevis        

        //double p_check = pressure-p;
        //std::cout<<p_check<<std::endl;

        srl::sleep(timestep);
        }

    srl::sleep(5);

*/






/*
////////////////////////////////////////////////////////////////
//PINCH thumb and index

    pressure = 1000; //1500 easy
    p_thumb = 1050; //1500 easy

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);


        //index flexion
        vc.setSinglePressure(6, 0.95*p);  //middle index       
        vc.setSinglePressure(7, 0.84*p);  //bottom index      
        vc.setSinglePressure(8, p);  //top index  

        //thumb flexion
        vc.setSinglePressure(12, 0.8*pt);  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*pt);  //top thumb flexor
        vc.setSinglePressure(10, pt);  //opponens pollicis
        vc.setSinglePressure(9, 0.6*pt);  //abductor pollicis brevis               

        //std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(10);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);


        //index relax
        vc.setSinglePressure(6, 0.95*(pressure-p) );  //middle index       
        vc.setSinglePressure(7, 0.84*(pressure-p) );  //bottom index      
        vc.setSinglePressure(8, pressure-p);  //top index  

        //thumb relax
        vc.setSinglePressure(12, 0.8*(p_thumb-pt) );  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*(p_thumb-pt) );  //top thumb flexor
        vc.setSinglePressure(10, (p_thumb-pt) );  //opponens pollicis
        vc.setSinglePressure(9, 0.6*(p_thumb-pt) );  //abductor pollicis brevis        

        //double p_check = pressure-p;
        //std::cout<<p_check<<std::endl;

        srl::sleep(timestep);
        }

    srl::sleep(5);
*/





/*
////////////////////////////////////////////////////////////////
//GRASP bottle, sphere, 

//set lead pressure in mbar
    pressure = 1600; //1500 easy
    p_thumb = 1500; //2000 max

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring flexion
        vc.setSinglePressure(0, 0.95*p);  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*p);  //bottom pinky and ring        
        vc.setSinglePressure(2, p);  //top pinky and ring         
        
        //middle flexion
        vc.setSinglePressure(3, 0.95*0.95*p);  //middle middle finger       
        vc.setSinglePressure(4, 0.95*0.84*p);  //bottom middle finger      
        vc.setSinglePressure(5, 0.95*p);  //top middle finger  

        //index flexion
        vc.setSinglePressure(6, 0.9*0.95*p);  //middle index       
        vc.setSinglePressure(7, 0.9*0.84*p);  //bottom index      
        vc.setSinglePressure(8, 0.9*p);  //top index               

        vc.setSinglePressure(10, 1.2*pt);  //opponens pollicis

        //std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }
    
    srl::sleep(0.2);

    duration = 5;
    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //thumb flexion
        vc.setSinglePressure(12, 0.8*pt);  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*pt);  //top thumb flexor
        // vc.setSinglePressure(10, 0.9*pt);  //opponens pollicis
        vc.setSinglePressure(9, 0.7*pt);  //abductor pollicis brevis               

        //std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }


    srl::sleep(10);

    duration = 10;
    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring relax
        vc.setSinglePressure(0, 0.95*(pressure-p) );  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*(pressure-p) );  //bottom pinky and ring        
        vc.setSinglePressure(2, pressure-p);  //top pinky and ring         
        
        //middle relax
        vc.setSinglePressure(3, 0.95*0.95*(pressure-p) );  //middle middle finger       
        vc.setSinglePressure(4, 0.95*0.84*(pressure-p) );  //bottom middle finger      
        vc.setSinglePressure(5, 0.95*(pressure-p) );  //top middle finger  

        //index relax
        vc.setSinglePressure(6, 0.9*0.95*(pressure-p) );  //middle index       
        vc.setSinglePressure(7, 0.9*0.84*(pressure-p) );  //bottom index      
        vc.setSinglePressure(8, 0.9*(pressure-p) );  //top index  

        //thumb relax
        vc.setSinglePressure(12, 0.8*(p_thumb-pt) );  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*(p_thumb-pt) );  //top thumb flexor
        vc.setSinglePressure(10, 1.2*(p_thumb-pt) );  //opponens pollicis
        vc.setSinglePressure(9, 0.7*(p_thumb-pt) );  //abductor pollicis brevis        

        //double p_check = pressure-p;
        //std::cout<<p_check<<std::endl;

        srl::sleep(timestep);
        }

    srl::sleep(5);

*/







/*
////////////////////////////////////////////////////////////////
//GRASP cube

//set lead pressure in mbar
    pressure = 1400; //1500 easy
    p_thumb = 1500; //2000 max

    duration = 5;
    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //thumb flexion
        vc.setSinglePressure(12, 0.8*pt);  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*pt);  //top thumb flexor
        vc.setSinglePressure(10, 1.1*pt);  //opponens pollicis
        vc.setSinglePressure(9, 0.9*pt);  //abductor pollicis brevis               

        srl::sleep(timestep);
    }

    srl::sleep(0.2);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring flexion
        vc.setSinglePressure(0, 0.95*p);  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*p);  //bottom pinky and ring        
        vc.setSinglePressure(2, p);  //top pinky and ring         
        
        //middle flexion
        vc.setSinglePressure(3, 0.85*0.95*p);  //middle middle finger       
        vc.setSinglePressure(4, 0.85*0.84*p);  //bottom middle finger      
        vc.setSinglePressure(5, 0.85*p);  //top middle finger  

        //index flexion
        vc.setSinglePressure(6, 0.85*0.95*p);  //middle index       
        vc.setSinglePressure(7, 0.85*0.84*p);  //bottom index      
        vc.setSinglePressure(8, 0.85*p);  //top index               

        //std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }
    
    srl::sleep(10);


    duration = 10;
    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring relax
        vc.setSinglePressure(0, 0.95*(pressure-p) );  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*(pressure-p) );  //bottom pinky and ring        
        vc.setSinglePressure(2, pressure-p);  //top pinky and ring         
        
        //middle relax
        vc.setSinglePressure(3, 0.85*0.95*(pressure-p) );  //middle middle finger       
        vc.setSinglePressure(4, 0.85*0.84*(pressure-p) );  //bottom middle finger      
        vc.setSinglePressure(5, 0.85*(pressure-p) );  //top middle finger  

        //index relax
        vc.setSinglePressure(6, 0.85*0.95*(pressure-p) );  //middle index       
        vc.setSinglePressure(7, 0.85*0.84*(pressure-p) );  //bottom index      
        vc.setSinglePressure(8, 0.85*(pressure-p) );  //top index  

        //thumb relax
        vc.setSinglePressure(12, 0.8*(p_thumb-pt) );  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*(p_thumb-pt) );  //top thumb flexor
        vc.setSinglePressure(10, 1.1*(p_thumb-pt) );  //opponens pollicis
        vc.setSinglePressure(9, 0.8*(p_thumb-pt) );  //abductor pollicis brevis        

        srl::sleep(timestep);
        }

    srl::sleep(5);

*/



/*
////////////////////////////////////////////////////////////////
//GRASP load-cell cylinder 45mm

//set lead pressure in mbar
    pressure = 1800; //1500 easy
    p_thumb = 1500; //2000 max

    duration = 5;
    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //thumb flexion
        vc.setSinglePressure(12, 0.8*pt);  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*pt);  //top thumb flexor
        vc.setSinglePressure(10, 1.1*pt);  //opponens pollicis
        vc.setSinglePressure(9, 0.9*pt);  //abductor pollicis brevis               

        srl::sleep(timestep);
    }

    srl::sleep(0.2);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring flexion
        vc.setSinglePressure(0, 0.95*p);  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*p);  //bottom pinky and ring        
        vc.setSinglePressure(2, p);  //top pinky and ring         
        
        //middle flexion
        vc.setSinglePressure(3, 0.9*0.95*p);  //middle middle finger       
        vc.setSinglePressure(4, 0.9*0.84*p);  //bottom middle finger      
        vc.setSinglePressure(5, 0.9*p);  //top middle finger  

        //index flexion
        vc.setSinglePressure(6, 0.9*0.95*p);  //middle index       
        vc.setSinglePressure(7, 0.9*0.84*p);  //bottom index      
        vc.setSinglePressure(8, 0.9*p);  //top index               

        //std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }
    
    srl::sleep(10);


    duration = 10;
    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring relax
        vc.setSinglePressure(0, 0.95*(pressure-p) );  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*(pressure-p) );  //bottom pinky and ring        
        vc.setSinglePressure(2, pressure-p);  //top pinky and ring         
        
        //middle relax
        vc.setSinglePressure(3, 0.9*0.95*(pressure-p) );  //middle middle finger       
        vc.setSinglePressure(4, 0.9*0.84*(pressure-p) );  //bottom middle finger      
        vc.setSinglePressure(5, 0.9*(pressure-p) );  //top middle finger  

        //index relax
        vc.setSinglePressure(6, 0.9*0.95*(pressure-p) );  //middle index       
        vc.setSinglePressure(7, 0.9*0.84*(pressure-p) );  //bottom index      
        vc.setSinglePressure(8, 0.9*(pressure-p) );  //top index  

        //thumb relax
        vc.setSinglePressure(12, 0.8*(p_thumb-pt) );  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*(p_thumb-pt) );  //top thumb flexor
        vc.setSinglePressure(10, 1.1*(p_thumb-pt) );  //opponens pollicis
        vc.setSinglePressure(9, 0.8*(p_thumb-pt) );  //abductor pollicis brevis        

        srl::sleep(timestep);
        }

    srl::sleep(5);

    */







///////////////////////////////////////////////////////////////
//Range of motion test

//set lead pressure in mbar
    pressure = 1900; //1500 easy
    duration = 10;

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring flexion
        vc.setSinglePressure(0, 0.95*p);  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*p);  //bottom pinky and ring        
        vc.setSinglePressure(2, p);  //top pinky and ring         

        //index flexion
        //vc.setSinglePressure(6, 0.95*p);  //middle index       
        //vc.setSinglePressure(7, 0.84*p);  //bottom index      
        //vc.setSinglePressure(8, p);  //top index               

        //std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }
    
    srl::sleep(10);


    
    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring relax
        vc.setSinglePressure(0, 0.95*(pressure-p) );  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*(pressure-p) );  //bottom pinky and ring        
        vc.setSinglePressure(2, pressure-p);  //top pinky and ring          

        //index relax
        //vc.setSinglePressure(6, 0.95*(pressure-p) );  //middle index       
        //vc.setSinglePressure(7, 0.84*(pressure-p) );  //bottom index      
        //vc.setSinglePressure(8, (pressure-p) );  //top index         

        srl::sleep(timestep);
        }

    srl::sleep(10);


    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        
        //Extensor pinky
        vc.setSinglePressure(13, p);              

        //Extensor middle and index
        //vc.setSinglePressure(14, p);              

        //std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(10);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        
        //Extensor middle and index relax
        vc.setSinglePressure(13, pressure-p );
        
        //Extensor middle and index relax
        //vc.setSinglePressure(14, pressure-p );

        srl::sleep(timestep);
        }

    srl::sleep(5);




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
