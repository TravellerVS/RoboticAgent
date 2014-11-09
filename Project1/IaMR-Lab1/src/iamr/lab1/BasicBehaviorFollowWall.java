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

    private double idealDistance = BasicSensors.DISTANCE_IDEAL;
    
    BasicBehaviorFollowWall(BasicMovements movements, BasicSensors sensors)
    {
        super(movements, sensors);
    }

    @Override
    protected boolean TestConitions() {
        if(    (distSensorCenter <= BasicSensors.DISTANCE_FAR )
            || (distSensorRight <= BasicSensors.DISTANCE_FAR )
            || (distSensorLeft <= BasicSensors.DISTANCE_FAR ))
            return true;
        return false;
    }

    @Override
    protected void DoBehavior() {        
        boolean direction = (distSensorLeft < distSensorRight) ? BasicMovements.ANTICLOCKWISE : BasicMovements.CLOCKWISE;        
        boolean wheel = (distSensorLeft < distSensorRight) ? BasicMovements.RIGHT : BasicMovements.LEFT;   
        double distance = (distSensorLeft<distSensorRight) ? distSensorLeft : distSensorRight ;
        if(distSensorCenter <= BasicSensors.DISTANCE_NEAR && distSensorCenter >= BasicSensors.DISTANCE_VERY_CLOSE)
            movements.RotateAroundWheel(BasicMovements.NORMAL_SPEED, wheel);
        else {
//            double percentLeft = distSensorLeft/idealDistance;
//            double percentRight = distSensorRight/idealDistance;
            
            //direction = (distance<idealDistance || distSensorCenter < BasicSensors.DISTANCE_NEAR) ? !direction : direction ;
            
            if(distance>BasicSensors.DISTANCE_NEAR )
                movements.MoveInArc(BasicMovements.NEAR_WALL_SPEED, direction, BasicMovements.HARD_TURN_RADIUS/2.0); 
            else
                movements.MoveInArc(BasicMovements.NEAR_WALL_SPEED, direction, BasicMovements.MEDIUM_TURN_RADIUS);              
        
//            double distance = (distSensorLeft<distSensorRight) ? distSensorLeft : distSensorRight ;
//            if(distance<idealDistance || distSensorCenter < BasicSensors.DISTANCE_NEAR){
//                distance = BasicSensors.DISTANCE_MEDIUM;
//                direction = !direction;
//            }else{
//                distance = (((Math.abs(distance)-2.0)*(-1))+2.0);
//            }
//            movements.MoveInArc(BasicMovements.NEAR_WALL_SPEED, direction, Math.abs(distance)); 
//            
        }
    }
    
}
