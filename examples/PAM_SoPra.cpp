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
    


    const int pressure = 2000;  // 90 degrees = 2bars


    double duration = 4;
    double timestep = 0.01;
    double p;

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
   
        vc.setSinglePressure(1, p);  //bottom 1      
        vc.setSinglePressure(2, p);  //bottom 2  
        //vc.setSinglePressure(3, p);  //bottom 3         
        vc.setSinglePressure(4, p);  //top 1       
        vc.setSinglePressure(5, p);  //top 2      
        //vc.setSinglePressure(6, p);  //top 3               

        //vc.setSinglePressure(1, pressure-p);  //bottom 1      
        //vc.setSinglePressure(2, pressure-p);  //bottom 2  
        //vc.setSinglePressure(3, pressure-p);  //bottom 3         
        //vc.setSinglePressure(4, pressure-p);  //top 1       
        //vc.setSinglePressure(5, pressure-p);  //top 2      
        //vc.setSinglePressure(6, pressure-p);  //top 3 


        std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(1);


    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
   
        //vc.setSinglePressure(1, p);  //bottom 1      
        //vc.setSinglePressure(2, p);  //bottom 2  
        vc.setSinglePressure(3, p);  //bottom 3         
        //vc.setSinglePressure(4, p);  //top 1       
        //vc.setSinglePressure(5, p);  //top 2      
        vc.setSinglePressure(6, p);  //top 3               

        vc.setSinglePressure(1, pressure-p);  //bottom 1      
        //vc.setSinglePressure(2, pressure-p);  //bottom 2  
        //vc.setSinglePressure(3, pressure-p);  //bottom 3         
        vc.setSinglePressure(4, pressure-p);  //top 1       
        //vc.setSinglePressure(5, pressure-p);  //top 2      
        //vc.setSinglePressure(6, pressure-p);  //top 3 


        std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(1);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
   
        vc.setSinglePressure(1, p);  //bottom 1      
        //vc.setSinglePressure(2, p);  //bottom 2  
        //vc.setSinglePressure(3, p);  //bottom 3         
        vc.setSinglePressure(4, p);  //top 1       
        //vc.setSinglePressure(5, p);  //top 2      
        //vc.setSinglePressure(6, p);  //top 3               

        //vc.setSinglePressure(1, pressure-p);  //bottom 1      
        vc.setSinglePressure(2, pressure-p);  //bottom 2  
        //vc.setSinglePressure(3, pressure-p);  //bottom 3         
        //vc.setSinglePressure(4, pressure-p);  //top 1       
        vc.setSinglePressure(5, pressure-p);  //top 2      
        //vc.setSinglePressure(6, pressure-p);  //top 3 


        std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(1);


    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
   
        //vc.setSinglePressure(1, p);  //bottom 1      
        vc.setSinglePressure(2, p);  //bottom 2  
        //vc.setSinglePressure(3, p);  //bottom 3         
        //vc.setSinglePressure(4, p);  //top 1       
        vc.setSinglePressure(5, p);  //top 2      
        //vc.setSinglePressure(6, p);  //top 3               

        //vc.setSinglePressure(1, pressure-p);  //bottom 1      
        //vc.setSinglePressure(2, pressure-p);  //bottom 2  
        vc.setSinglePressure(3, pressure-p);  //bottom 3         
        //vc.setSinglePressure(4, pressure-p);  //top 1       
        //vc.setSinglePressure(5, pressure-p);  //top 2      
        vc.setSinglePressure(6, pressure-p);  //top 3 


        std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(1);


    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
   
        //vc.setSinglePressure(1, p);  //bottom 1      
        //vc.setSinglePressure(2, p);  //bottom 2  
        //vc.setSinglePressure(3, p);  //bottom 3         
        //vc.setSinglePressure(4, p);  //top 1       
        //vc.setSinglePressure(5, p);  //top 2      
        //vc.setSinglePressure(6, p);  //top 3               

        vc.setSinglePressure(1, pressure-p);  //bottom 1      
        vc.setSinglePressure(2, pressure-p);  //bottom 2  
        //vc.setSinglePressure(3, pressure-p);  //bottom 3         
        vc.setSinglePressure(4, pressure-p);  //top 1       
        vc.setSinglePressure(5, pressure-p);  //top 2      
        //vc.setSinglePressure(6, pressure-p);  //top 3 


        std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(1);
/*
    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
   
        //vc.setSinglePressure(1, p);  //bottom 1      
        //vc.setSinglePressure(2, p);  //bottom 2  
        vc.setSinglePressure(3, p);  //bottom 3         
        vc.setSinglePressure(4, p);  //top 1       
        //vc.setSinglePressure(5, p);  //top 2      
        //vc.setSinglePressure(6, p);  //top 3               

        //vc.setSinglePressure(1, pressure-p);  //bottom 1      
        vc.setSinglePressure(2, pressure-p);  //bottom 2  
        //vc.setSinglePressure(3, pressure-p);  //bottom 3         
        //vc.setSinglePressure(4, pressure-p);  //top 1       
        //vc.setSinglePressure(5, pressure-p);  //top 2      
        vc.setSinglePressure(6, pressure-p);  //top 3 


        std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(1);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
   
        vc.setSinglePressure(1, p);  //bottom 1      
        //vc.setSinglePressure(2, p);  //bottom 2  
        //vc.setSinglePressure(3, p);  //bottom 3         
        //vc.setSinglePressure(4, p);  //top 1       
        vc.setSinglePressure(5, p);  //top 2      
        //vc.setSinglePressure(6, p);  //top 3               

        //vc.setSinglePressure(1, pressure-p);  //bottom 1      
        //vc.setSinglePressure(2, pressure-p);  //bottom 2  
        //vc.setSinglePressure(3, pressure-p);  //bottom 3         
        vc.setSinglePressure(4, pressure-p);  //top 1       
        //vc.setSinglePressure(5, pressure-p);  //top 2      
        //vc.setSinglePressure(6, pressure-p);  //top 3 


        std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(1);

    for (double time = 0; time <= duration; time += timestep) {
        p = pressure * (time / duration);
   
        //vc.setSinglePressure(1, p);  //bottom 1      
        //vc.setSinglePressure(2, p);  //bottom 2  
        //vc.setSinglePressure(3, p);  //bottom 3         
        //vc.setSinglePressure(4, p);  //top 1       
        //vc.setSinglePressure(5, p);  //top 2      
        //vc.setSinglePressure(6, p);  //top 3               

        vc.setSinglePressure(1, pressure-p);  //bottom 1      
        //vc.setSinglePressure(2, pressure-p);  //bottom 2  
        vc.setSinglePressure(3, pressure-p);  //bottom 3         
        //vc.setSinglePressure(4, pressure-p);  //top 1       
        vc.setSinglePressure(5, pressure-p);  //top 2      
        //vc.setSinglePressure(6, pressure-p);  //top 3 


        std::cout<<p<<std::endl;
        srl::sleep(timestep);
    }

    srl::sleep(1);


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
