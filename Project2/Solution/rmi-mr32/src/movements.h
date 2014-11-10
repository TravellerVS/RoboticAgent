#ifndef __RMI_P2_MOVEMENTS
#define __RMI_P2_MOVEMENTS

void rotateRel_naive(double deltaAngle);
void movement_go_forward(int speed);
void movement_stop();
void movement_rotate(int speed, bool direction);
void movement_travel_in_curve_radius(int speed, double radius);
void movement_travel_in_curve_radius_direction(int speed, double radius, bool direction);

#endif
