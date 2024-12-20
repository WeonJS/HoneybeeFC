
#include "HoneybeeFramework.h"

using namespace HoneybeeFramework;

enum ActuatorPosition
{
    FORWARD_LEFT,
    FORWARD_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
};

class ThrustActuator : public SubSystem
{
    public:
        void init();
        void update() override;
        Honeybee_ErrType attach_servo(ActuatorPosition pos, int pin);
        Honeybee_ErrType attach_propeller(ActuatorPosition pos, int pin);
        void set_thrust(ActuatorPosition pos, int thrust);
        void get_thrust(ActuatorPosition pos);

    private:
        int thrust : 12;
        Servo servos[4];
        Propeller props[4];
};