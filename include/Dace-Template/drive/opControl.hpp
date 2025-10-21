#pragma once
#include "main.h"

namespace dace{

    enum class Stick { Left, Right };

    //Drive control types
    void arcade_drive(int deadzone = 10);
    void tank_drive(int deadzone = 10);
    void single_stick_drive(Stick which = Stick::Right, int deadzone = 10);

} //namespace dace