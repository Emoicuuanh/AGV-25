#ifndef DIMENSION_H_
#define DIMENSION_H_

#define PI 3.141592
#define WHEEL_RADIUS                     0.075                              // meter
#define WHEEL_SEPARATION                 0.523                              // meter
#define ENCODER_MIN  0                   -2147483648                        // raw 
#define ENCODER_MAX                      2147483648                         // raw
#define ENCODER_RESOLUTION               150                                // pulse per round
#define TICK2RAD                         (2 * PI) / ENCODER_RESOLUTION
#define LEFT                             0
#define RIGHT                            1
#define WHEEL_NUM                        2

#endif