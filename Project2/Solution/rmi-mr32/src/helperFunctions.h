#ifndef __RMI_FP_HELPER_FUNCTIONS_H
#define __RMI_FP_HELPER_FUNCTIONS_H


/** \brief Normalizes angle to [-pi,+pi]
 *  \param *angle - pointer to double value of unnormalized angle
 * */
void normalize_angle(double *angle);
/** \brief Normalizes angle to [0,2*pi]
 *  \param *angle - pointer to double value of unnormalized angle
 * */
void normalize_angle_to_positive(double *angle);

/** \brief Calulates normalized absolute difference between two angles
 *  \return Returns double value - normalized absolute difference between two angles
 */
double abs_angle_difference(double angle1, double angle2);

/** \brief Calulates normalized difference between two angles that can be positive or negative
 * \details The function calculates this asuming that the result assuming that the
 * wanted result is the distance from angle1 to angle2
 * *  \param angle1 - start angle
 * *  \param angle2 - end angle
 *  \return Returns double value - normalized difference between two angles, can be positive or negative
 */
double angle_difference(double angle1, double angle2);

double deg_to_rad(double degree);
double rad_to_deg(double rad);
double angle_between_points(double x1, double y1,double x2, double y2);
double distance_between_points(double x1, double y1,double x2, double y2);

#endif
