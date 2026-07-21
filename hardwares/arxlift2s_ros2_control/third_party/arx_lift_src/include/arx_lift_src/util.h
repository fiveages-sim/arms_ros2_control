//
// Created by arx4060 on 24-12-4.
//

#pragma once

#include <cmath>

template <typename T>
T limit(const T& v, const T& lb, const T& ub)
{
    if (v < lb)
    {
        return lb;
    }
    else if (v > ub)
    {
        return ub;
    }
    else
    {
        return v;
    }
}

double ramp(double goal, double current, double ramp_k)
{
    float retval = 0.0f;
    float delta = 0.0f;
    delta = goal - current;
    if (delta > 0)
    {
        if (delta > ramp_k)
        {
            current += ramp_k;
        }
        else
        {
            current += delta;
        }
    }
    else
    {
        if (delta < -ramp_k)
        {
            current += -ramp_k;
        }
        else
        {
            current += delta;
        }
    }
    retval = current;
    return retval;
}
