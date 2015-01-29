//#define _USE_MATH_DEFINES
#include <math.h>
#include "helperFunctions.h"

void normalize_angle(double *angle){
	double x = cos(*angle);
	double y = sin(*angle);
	*angle = atan2(y,x);
}

void normalize_angle_to_positive(double *angle){
	normalize_angle(angle);
	if(*angle<0){
		*angle += 2*M_PI;
	}
}

double abs_angle_difference(double angle1, double angle2){
	double difference = angle_difference(angle1,angle2);
	return  fabs(difference);
}

double angle_difference(double angle1, double angle2){
	normalize_angle_to_positive(&angle1);
	normalize_angle_to_positive(&angle2);
	double difference = angle2-angle1;
	normalize_angle(&difference);
	return difference;
}

double deg_to_rad(double degree){
	return (degree*M_PI)/180.0;
}
double rad_to_deg(double rad){
	return (rad*180)/M_PI;
}
double angle_between_points(double x1, double y1,double x2, double y2){
	return atan2((y2-y1), (x2-x1));
}

double angle_to_dest(double x1, double y1, double t1,double x2, double y2){
	double angle = angle_between_points(x1, y1,x2, y2);
	return angle_difference(t1, angle);
}

double distance_between_points(double x1, double y1,double x2, double y2){
	double dx = (x2-x1);
	double dy = (y2-y1);
	return sqrt(dx*dx+dy*dy);
}
void new_points_from_distance_and_angle(double x_old, double y_old, double *x_new, double *y_new, double angle, double distace){
	double dx = cos(angle)*distace;
	double dy = sin(angle)*distace;	
	*x_new = x_old + dx;
	*y_new = y_old + dy;
}

