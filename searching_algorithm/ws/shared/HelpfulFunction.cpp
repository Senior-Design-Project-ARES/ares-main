#include "HelpfulFunction.h"

double unwrapAngle(double x){
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0)
        x += 2 * M_PI;
    return x - M_PI;
}