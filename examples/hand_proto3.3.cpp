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
// Kapandji test

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





////////////////////////////////////////////////////////////////

/*
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

    srl::sleep(3);

*/


/*

////////////////////////////////////////////////////////////////
//PINCH thumb and pinky

    pressure = 1000; //1500 easy
    p_thumb = 2000; //1500 easy

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //Pinky & ring flexion
        vc.setSinglePressure(0, 0.95*p);  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*p);  //bottom pinky and ring        
        vc.setSinglePressure(2, p);  //top pinky and ring     

        //thumb flexion
        vc.setSinglePressure(12, 0.95*pt);  //bottom thumb flexor      
        vc.setSinglePressure(11, pt);  //top thumb flexor
        vc.setSinglePressure(10, pt);  //opponens pollicis
        vc.setSinglePressure(9, pt);  //abductor pollicis brevis               

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

        //thumb relax
        vc.setSinglePressure(12, 0.95*(p_thumb-pt) );  //bottom thumb flexor      
        vc.setSinglePressure(11, (p_thumb-pt) );  //top thumb flexor
        vc.setSinglePressure(10, (p_thumb-pt) );  //opponens pollicis
        vc.setSinglePressure(9, (p_thumb-pt) );  //abductor pollicis brevis        

        srl::sleep(timestep);
        }

    srl::sleep(5);


*/

// /*

////////////////////////////////////////////////////////////////
//PINCH thumb and ring
//change cupling

    pressure = 1000; //1500 easy
    p_thumb = 1700; //1500 easy

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //ring flexion
        vc.setSinglePressure(0, 0.95*p);  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*p);  //bottom pinky and ring        
        vc.setSinglePressure(2, p);  //top pinky and ring     

        //thumb flexion
        vc.setSinglePressure(12, 0.9*pt);  //bottom thumb flexor      
        vc.setSinglePressure(11, pt);  //top thumb flexor
        vc.setSinglePressure(10, pt);  //opponens pollicis
        vc.setSinglePressure(9, 0.7*pt);  //abductor pollicis brevis               

        srl::sleep(timestep);
    }

    srl::sleep(10);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //ring relax
        vc.setSinglePressure(0, 0.95*(pressure-p) );  //middle pinky and ring      
        vc.setSinglePressure(1, 0.84*(pressure-p) );  //bottom pinky and ring        
        vc.setSinglePressure(2, pressure-p);  //top pinky and ring       

        //thumb relax
        vc.setSinglePressure(12, 0.9*(p_thumb-pt) );  //bottom thumb flexor      
        vc.setSinglePressure(11, (p_thumb-pt) );  //top thumb flexor
        vc.setSinglePressure(10, (p_thumb-pt) );  //opponens pollicis
        vc.setSinglePressure(9, 0.7*(p_thumb-pt) );  //abductor pollicis brevis        

        srl::sleep(timestep);
        }

    srl::sleep(5);



    // */










////////////////////////////////////////////////////////////////
//PINCH thumb and middle

    pressure = 1100; //1500 easy
    p_thumb = 1300; //1500 easy

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //middle flexion
        vc.setSinglePressure(3, 0.95*p);  //middle middle finger       
        vc.setSinglePressure(4, 0.84*p);  //bottom middle finger      
        vc.setSinglePressure(5, p);  //top middle finger   

        //thumb flexion
        vc.setSinglePressure(12, 0.8*pt);  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*pt);  //top thumb flexor
        vc.setSinglePressure(10, pt);  //opponens pollicis
        vc.setSinglePressure(9, 0.5*pt);  //abductor pollicis brevis               

        srl::sleep(timestep);
    }

    srl::sleep(10);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
        pt = p_thumb * (time / duration);

        //middle relax
        vc.setSinglePressure(3, 0.95*(pressure-p) );  //middle middle finger       
        vc.setSinglePressure(4, 0.84*(pressure-p) );  //bottom middle finger      
        vc.setSinglePressure(5, pressure-p);  //top middle finger  

        //thumb relax
        vc.setSinglePressure(12, 0.8*(p_thumb-pt) );  //bottom thumb flexor      
        vc.setSinglePressure(11, 0.9*(p_thumb-pt) );  //top thumb flexor
        vc.setSinglePressure(10, (p_thumb-pt) );  //opponens pollicis
        vc.setSinglePressure(9, 0.5*(p_thumb-pt) );  //abductor pollicis brevis        

        srl::sleep(timestep);
        }

    srl::sleep(5);










////////////////////////////////////////////////////////////////
//PINCH thumb and index

    pressure = 1000; //1500 easy
    p_thumb = 1000; //1500 easy

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
        //vc.setSinglePressure(9, 0.7*pt);  //abductor pollicis brevis               

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
        //vc.setSinglePressure(9, 0.7*(p_thumb-pt) );  //abductor pollicis brevis        

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




}
