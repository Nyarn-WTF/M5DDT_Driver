//
// Created by wn on 5/28/23.
//

#include "Patlite.h"
#include <M5Stack.h>
#include <Arduino.h>

Patlite::Patlite(){
    relay = new MODULE_4RELAY();
    while(!relay->begin(&Wire, MODULE_4RELAY_ADDR, 21, 22, 200000L))
        delay(1000);
    relay->setRelay(0,true);
}

void Patlite::set_red(bool sts){
    relay->setRelay(1,sts);
}

void Patlite::set_green(bool sts){
    relay->setRelay(2,sts);
}

void Patlite::set_yellow(bool sts){
    relay->setRelay(3,sts);
}