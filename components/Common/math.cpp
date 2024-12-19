#include "math.h"

float clamp(float value, float min, float max) 
{
    if (value < min) value = min;
    if (value > max) value = max;

    return value;
}

float normalize(float value, float min, float max) 
{
    return (value - min) / (max - min);
}