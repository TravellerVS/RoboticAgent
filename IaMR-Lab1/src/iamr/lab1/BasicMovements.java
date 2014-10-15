/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

import ciberIF.ciberIF;

/**
 *
 * @author vedran
 */
public class BasicMovements {
    
    public static double SLOW_SPEED = 0.03;
    public static double FAST_SPEED = 0.13;
    public static double NORMAL_SPEED = 0.05;
    
    public static double LIGHT_TURN = 0.2;
    public static double MEDIUM_TURN = 0.6;
    public static double HARD_TURN = 0.9;
    
    public static double LIGHT_TURN_RADIUS = 10;
    public static double MEDIUM_TURN_RADIUS = 4;
    public static double HARD_TURN_RADIUS = 1;
    
    public static boolean RIGHT = true;
    public static boolean LEFT = false;
    
    public static boolean CLOCKWISE = true;
    public static boolean ANTICLOCKWISE = false;
    
    
    private ciberIF cif;
    private double robotWidth = 1;
    BasicMovements(ciberIF cif) {
        this.cif = cif;
    }
    /**
     * 
     * @param speed - desired speed of the robot
     */
    public void Move(double speed){
       cif.DriveMotors(speed, speed);
    }
    
    /**
     * @param speed - desired speed of the robot
     * @param percent - percent difference between speed o wheels positive values turn right, negative turn left
     */
    public void MoveInTurn(double speed, double percent){
        double speedLeft = speed*(1.0 + percent);
        double speedRight = speed*(1.0 - percent);
        cif.DriveMotors(speedLeft, speedLeft);
    }
    
    /**
     * @param speed - desired speed of the robot
     * @param percent - percent difference between speed o wheels
     * @param direction - if true turn right, else turn left
     */
    public void MoveInTurn(double speed, double percent, Boolean direction){
       double speedBig = speed*(1.0 + percent);
       double speedSmall = speed*(1.0 - percent);
       if(direction == CLOCKWISE){
            cif.DriveMotors(speedBig, speedSmall);
        }else{
            cif.DriveMotors(speedSmall, speedBig);
        }
    }
    
    public void Stop(){
        cif.DriveMotors(0, 0);
    }
    /**
     * @param speed - speed of rotation
     */
    public void Rotate(double speed){
        Rotate(speed, true);
    }
    /**
     * @param speed - speed of rotation
     * @param clockwise - if true turn clockwise else turn anticlockwise
     */
    public void Rotate (double speed, Boolean clockwise)
    {        
        if(clockwise == CLOCKWISE){
            cif.DriveMotors(speed, -speed);
        }else{
            cif.DriveMotors(-speed, speed);
        }
    }
    
    /**
     * @param speed - speed of rotation
     * @param wheel - if true turn around right wheel, else turn around left wheel
     */
    public void RotateAroundWheel (double speed, Boolean wheel)
    {
        if(wheel == RIGHT){
            //rotate around right wheel
            cif.DriveMotors(speed, 0);
        }else{
            //rotate around left wheel
            cif.DriveMotors(0, speed);
        }
    }
    
    /**
     * @param speed - speed of the robot
     * @param direction - if true turn right, else turn left
     * @param radius - radius of a circle containing the intended path
     */
    public void MoveInArc (double speed, Boolean direction, double radius)
    {    
        double radiusLarge = radius+robotWidth/2;
        double radiusSmall = radius-robotWidth/2;
        double speedLarge = speed*(radius/radiusLarge);
        double speedSmall = speed*(radius/radiusSmall);
        if(direction == RIGHT){
            cif.DriveMotors(speedLarge, speedSmall);
        }else{
            cif.DriveMotors(speedSmall, speedLarge);
        }
    }   
}
