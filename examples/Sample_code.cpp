#include "mobilerack-interface/ValveController.h"

int main() {

    std::vector<int> map = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};  //select valves 0, 1, 2, 3, ... 15
    ValveController vc{"192.168.0.100", map, 2000}; //last parameter sets max. pressure in mBar
    
// Prototype 3, sample grasp with flexion and relaxation of all fingers

    double duration = 8; //defines time for the flexion/relaxation motion in seconds
    const double timestep = 0.01; 

    //defines pressure variables for each finger
    double plr;
    double pm;
    double pi;
    double pt;

    //set lead pressures in mbar for each finger
    //adjust these values depending on the extent of motion desired for the individual fingers
    int pl_lr = 1800;   //little & ring finger are coupled in this configuration
    int pl_m = 1800;    //middle finger
    int pl_i = 1800;    //index finger
    int pl_t = 1200;    //thumb

    //set pressure ratios for the flexor actuators of each finger with respect to the lead pressure
    const double medi = 0.95;
    const double prox = 0.85;
    const double dist = 1;
    const double t_prox = 0.8;
    const double t_dist = 0.9;
    double t_op = 1;      //adjust depending on desired opposing motion of thumb
    double t_apb = 0.7;   //adjust depending on desired opposing motion of thumb



    for (double time = 0; time <= duration; time += timestep) {
        //linear increase of pressure variables until set lead pressure is reached
        plr = pl_lr * (time / duration);
        pm = pl_m * (time / duration);
        pi = pl_i * (time / duration);
        pt = pl_t * (time / duration);

        //little & ring flexion
        vc.setSinglePressure(0, medi*plr);  //middle tendon (phalanx medialis) little and ring finger     
        vc.setSinglePressure(1, prox*plr);  //bottom tendon (phalanx proximalis) little and ring finger        
        vc.setSinglePressure(2, dist*plr);  //top tendon (phalanx distalis) little and ring finger           
        
        //middle flexion
        vc.setSinglePressure(3, medi*pm);  //middle tendon (phalanx medialis) middle finger       
        vc.setSinglePressure(4, prox*pm);  //bottom tendon (phalanx proximalis) middle finger      
        vc.setSinglePressure(5, dist*pm);  //top (phalanx distalis) middle finger  

        //index flexion
        vc.setSinglePressure(6, medi*pi);  //middle tendon (phalanx medialis) index       
        vc.setSinglePressure(7, prox*pi);  //bottom tendon (phalanx proximalis) index      
        vc.setSinglePressure(8, dist*pi);  //top (phalanx distalis) index  

        //thumb flexion
        vc.setSinglePressure(12, t_prox*pt);  //bottom (phalanx proximalis) thumb flexor      
        vc.setSinglePressure(11, t_dist*pt);  //top (phalanx distalis) thumb flexor
        vc.setSinglePressure(10, t_op*pt);  //opponens pollicis
        vc.setSinglePressure(9, t_apb*pt);  //abductor pollicis brevis               

        srl::sleep(timestep);
    }

    srl::sleep(10); //defines holding time for grasp in seconds

    for (double time = 0; time <= duration; time += timestep) {
        plr = pl_lr * (time / duration);
        pm = pl_m * (time / duration);
        pi = pl_i * (time / duration);
        pt = pl_t * (time / duration);

        //Pinky & ring relax
        vc.setSinglePressure(0, medi*(pl_lr - plr) );  //middle tendon (phalanx medialis) little and ring finger       
        vc.setSinglePressure(1, prox*(pl_lr - plr) );  //bottom tendon (phalanx proximalis) little and ring finger          
        vc.setSinglePressure(2, dist*(pl_lr - plr) );  //top (phalanx distalis) little and ring finger          
        
        //middle relax
        vc.setSinglePressure(3, medi*(pl_m - pm) );  //middle tendon (phalanx medialis) middle finger       
        vc.setSinglePressure(4, prox*(pl_m - pm) );  //bottom tendon (phalanx proximalis) middle finger      
        vc.setSinglePressure(5, dist*(pl_m - pm) );  //top tendon (phalanx proximalis) middle finger  

        //index relax
        vc.setSinglePressure(6, medi*(pl_i - pl_i) );  //middle tendon (phalanx medialis) index       
        vc.setSinglePressure(7, prox*(pl_i - pl_i) );  //bottom tendon (phalanx proximalis) index      
        vc.setSinglePressure(8, dist*(pl_i - pl_i) );  //top (phalanx distalis) index  

        //thumb relax
        vc.setSinglePressure(12, t_prox*(pl_t - pt) );  //bottom tendon (phalanx proximalis) thumb flexor      
        vc.setSinglePressure(11, t_dist*(pl_t - pt) );  //top (phalanx distalis) thumb flexor
        vc.setSinglePressure(10, t_op*(pl_t - pt) );  //opponens pollicis
        vc.setSinglePressure(9, t_apb*(pl_t - pt) );  //abductor pollicis brevis        

        srl::sleep(timestep);
        }

}
