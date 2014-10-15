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
    private boolean defaultDirection;
    BasicBehaviorAvoidColision(BasicMovements movements, BasicSensors sensors)
    {
        super(movements, sensors);
        defaultDirection = (HelperFunctions.randBool()) ? BasicMovements.RIGHT : BasicMovements.LEFT ;
    }

    @Override
    protected boolean TestConitions() {
        if(distSensorCenter <= BasicSensors.DISTANCE_ALMOST_COLIDED || distSensorLeft <= BasicSensors.DISTANCE_ALMOST_COLIDED ||  distSensorRight <= BasicSensors.DISTANCE_ALMOST_COLIDED)
            return true;
        return false;
    }

    @Override
    protected void DoBehavior() {
        boolean direction = (distSensorLeft > distSensorRight) ? BasicMovements.ANTICLOCKWISE : BasicMovements.CLOCKWISE;
        boolean wheel = (distSensorLeft > distSensorRight) ? BasicMovements.LEFT : BasicMovements.RIGHT;
        if(distSensorRight >= BasicSensors.DISTANCE_NEAR || distSensorLeft >= BasicSensors.DISTANCE_NEAR)
            movements.Rotate(BasicMovements.NORMAL_SPEED, direction);
        else
            movements.Rotate(BasicMovements.NORMAL_SPEED, defaultDirection);       
//        movements.Rotate(BasicMovements.NORMAL_SPEED, direction);
            
    }
}
