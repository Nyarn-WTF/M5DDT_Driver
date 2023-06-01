//
// Created by wn on 5/28/23.
//

#ifndef M5DDT_DRIVER_PATLITE_H
#define M5DDT_DRIVER_PATLITE_H

#include "Module_4RELAY.h"

class Patlite {
private:
    bool power = false;
    bool red = false;
    bool yellow = false;
    bool green = false;
    MODULE_4RELAY *relay;
public:
    Patlite();
    void set_red(bool);
    void set_green(bool);
    void set_yellow(bool);
};

#endif //M5DDT_DRIVER_PATLITE_H
