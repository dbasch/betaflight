#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define M_PIf       3.14159265358979323846f
#define RAD    (M_PIf / 180.0f)
#define DEGREES_TO_DECIDEGREES(angle) (angle * 10)
#define DECIDEGREES_TO_DEGREES(angle) (angle / 10)
#define DECIDEGREES_TO_RADIANS(angle) ((angle / 10.0f) * 0.0174532925f)
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)
#define RADIANS_TO_DECIDEGREES(angle) (((angle) * 10.0f) / RAD)

int main(){
    int res = RADIANS_TO_DECIDEGREES(atan2(626,300)) / 10;
    printf("tan  %d\n", res);
}