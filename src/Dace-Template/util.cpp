#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

/*
Util
*/

void delay(int time){
    pros::delay(time);
}

namespace dace{


    void pidStandards(){
        setExitConfig(1.0, 1.0, 3000);
    }

}//namespace dace