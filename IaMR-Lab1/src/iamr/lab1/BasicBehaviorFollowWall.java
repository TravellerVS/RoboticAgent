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
        if(distSensorCenter <= BasicSensors.DISTANCE_NEAR && distSensorCenter >= BasicSensors.DISTANCE_VERY_CLOSE)
            movements.Rotate(BasicMovements.NORMAL_SPEED, direction);
        else {
            double percentLeft = distSensorLeft/idealDistance;
            double percentRight = distSensorRight/idealDistance;
            double percent;
            if(Math.abs(percentLeft) < Math.abs(percentRight)){
                percent = percentLeft*4;
                if( Math.abs(distSensorLeft/BasicSensors.DISTANCE_NEAR)> 0.8 ){
                    movements.MoveInArc(BasicMovements.NORMAL_SPEED, BasicMovements.ANTICLOCKWISE, BasicMovements.HARD_TURN_RADIUS);
                    return;
                }
            }else{
                percent = percentRight*4;
                if( Math.abs(distSensorRight/BasicSensors.DISTANCE_NEAR)> 0.8 ){
                    movements.MoveInArc(BasicMovements.NORMAL_SPEED, BasicMovements.ANTICLOCKWISE, BasicMovements.HARD_TURN_RADIUS);
                    return;
                }
            }            
            movements.MoveInTurn(BasicMovements.NORMAL_SPEED, percent);               
        }
    }
    
}
