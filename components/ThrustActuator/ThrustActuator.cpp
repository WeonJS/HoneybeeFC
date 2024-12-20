#include <stdio.h>
#include "ThrustActuator.h"

void ThrustActuator::init()
{
    
}

Honeybee_ErrType ThrustActuator::attach_servo(ActuatorPosition pos, int pin)
{
    servos[pos].init(180);
    servos[pos].attach(pin);
    return HONEYBEE_OK;
}

Honeybee_ErrType ThrustActuator::attach_propeller(ActuatorPosition pos, int pin)
{
    props[pos].init();
    props[pos].attach(pin);
    return HONEYBEE_OK;
}

void ThrustActuator::set_thrust(ActuatorPosition pos, int thrust)
{
    props[pos].set_thrust(thrust);
}

void ThrustActuator::get_thrust(ActuatorPosition pos)
{
    props[pos].get_thrust();
}

void ThrustActuator::update()
{
    for (int i = 0; i < 4; i++)
    {
        props[i].send_data();
    }
}