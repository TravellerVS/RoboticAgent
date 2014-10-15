/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

/**
 *
 * @author vedran
 */
public class BasicBehaviorAvoidColision extends BasicBehavior{
    
    BasicBehaviorAvoidColision(BasicMovements movements, BasicSensors sensors)
    {
        super(movements, sensors);
    }

    @Override
    protected boolean TestConitions() {
        if(distSensorCenter <= BasicSensors.DISTANCE_VERY_CLOSE || distSensorLeft <= BasicSensors.DISTANCE_VERY_CLOSE ||  distSensorRight <= BasicSensors.DISTANCE_VERY_CLOSE)
            return true;
        return false;
    }

    @Override
    protected void DoBehavior() {
        boolean direction = (distSensorLeft > distSensorRight) ? BasicMovements.ANTICLOCKWISE : BasicMovements.CLOCKWISE;
        boolean wheel = (distSensorLeft > distSensorRight) ? BasicMovements.LEFT : BasicMovements.RIGHT;
//        if((distSensorLeft >= BasicSensors.DISTANCE_NEAR ||  distSensorRight >= BasicSensors.DISTANCE_NEAR) && distSensorCenter >= BasicSensors.DISTANCE_NEAR)
//            movements.RotateAroundWheel(BasicMovements.NORMAL_SPEED, wheel);
//        else
//            movements.Rotate(BasicMovements.NORMAL_SPEED, direction);
        movements.Rotate(BasicMovements.NORMAL_SPEED, direction);
            
    }
}
