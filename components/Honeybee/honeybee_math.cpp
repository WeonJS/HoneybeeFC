#include "honeybee_math.h"
#include <cmath>

namespace honeybee_math {
    

    

    void Vector2::set_x(float x)
    {
        this->x = x;
    }

    void Vector2::set_y(float y)
    {
        this->y = y;
    }

    float Vector2::get_x()
    {
        return this->x;
    }

    float Vector2::get_y()
    {
        return this->y;
    }

    Vector2 operator+(Vector2 a, Vector2 b)
    {
        Vector2 ret;
        ret.set_x(a.get_x() + b.get_x());
        ret.set_y(a.get_y() + b.get_y());
        return ret;
    }

    Vector2 operator-(Vector2 a, Vector2 b)
    {
        Vector2 ret;
        ret.set_x(a.get_x() - b.get_x());
        ret.set_y(a.get_y() - b.get_y());
        return ret;
    }

    Vector2 operator*(Vector2 a, float b)
    {
        Vector2 ret;
        ret.set_x(a.get_x() * b);
        ret.set_y(a.get_y() * b);
        return ret;
    }

    Vector2 operator*(float a, Vector2 b)
    {
        Vector2 ret;
        ret.set_x(a * b.get_x());
        ret.set_y(a * b.get_y());
        return ret;
    }

    Vector2 operator/(Vector2 a, float b)
    {
        Vector2 ret;
        ret.set_x(a.get_x() / b);
        ret.set_y(a.get_y() / b);
        return ret;
    }

    // todo: map function for floats
    float map(float value, float in_min, float in_max, float out_min, float out_max)
    {
        float val = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return clamp(val, out_min, out_max);
    }

    float clamp(float value, float min, float max)
    {
        if (value < min)
            value = min;
        if (value > max)
            value = max;

        return value;
    }

    float Vector2::dot(Vector2 other)
    {
        return x * other.get_x() + y * other.get_y();
    }

    float normalize(float value, float min, float max)
    {
        return (value - min) / (max - min);
    }

    Vector2 Vector2::normal()
    {
        float magnitude = sqrt(this->x * this->x + this->y * this->y);
        Vector2 ret;
        ret.set_x(this->x / magnitude);
        ret.set_y(this->y / magnitude);
        return ret;
    }
}