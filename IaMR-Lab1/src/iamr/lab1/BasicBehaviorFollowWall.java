/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

/**
 *
 * @author vedran
 */
public class BasicBehaviorFollowWall extends BasicBehavior{

    private double idealDistance = (BasicSensors.DISTANCE_NEAR + BasicSensors.DISTANCE_VERY_CLOSE)*0.5;
    
    BasicBehaviorFollowWall(BasicMovements movements, BasicSensors sensors)
    {
        super(movements, sensors);
    }

    @Override
    protected boolean TestConitions() {
        if(    (distSensorCenter <= BasicSensors.DISTANCE_NEAR && distSensorCenter >= BasicSensors.DISTANCE_VERY_CLOSE)
            || (distSensorRight <= BasicSensors.DISTANCE_NEAR && distSensorRight >= BasicSensors.DISTANCE_VERY_CLOSE)
            || (distSensorLeft <= BasicSensors.DISTANCE_NEAR && distSensorLeft >= BasicSensors.DISTANCE_VERY_CLOSE))
            return true;
        return false;
    }

    @Override
    protected void DoBehavior() {        
        boolean direction = (distSensorLeft > distSensorRight) ? BasicMovements.ANTICLOCKWISE : BasicMovements.CLOCKWISE;        
        boolean wheel = (distSensorLeft > distSensorRight) ? BasicMovements.LEFT : BasicMovements.RIGHT;   
        if(distSensorCenter <= BasicSensors.DISTANCE_NEAR && distSensorCenter >= BasicSensors.DISTANCE_VERY_CLOSE)
            movements.RotateAroundWheel(BasicMovements.NORMAL_SPEED, wheel);
        else {
            double percentLeft = distSensorLeft/idealDistance;
            double percentRight = distSensorRight/idealDistance;
            double percent;
            if(Math.abs(percentLeft) < Math.abs(percentRight)){
                percent = percentLeft*3.0;
                if( Math.abs(distSensorLeft/BasicSensors.DISTANCE_NEAR)> 0.7 ){
                    movements.MoveInArc(BasicMovements.SLOW_SPEED, BasicMovements.ANTICLOCKWISE, BasicMovements.HARD_TURN_RADIUS);
                    return;
                }
            }else{
                percent = percentRight*3.0;
                if( Math.abs(distSensorRight/BasicSensors.DISTANCE_NEAR)> 0.7 ){
                    movements.MoveInArc(BasicMovements.SLOW_SPEED, BasicMovements.CLOCKWISE, BasicMovements.HARD_TURN_RADIUS);
                    return;
                }
            }            
            movements.MoveInTurn(BasicMovements.SLOW_SPEED, percent);               
        }
    }
    
}
