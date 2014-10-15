/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

/**
 *
 * @author vedran
 */
public class BasicBehaviorGoToBeacon extends BasicBehavior {

    private boolean defaultDirection;
    private double angleOfVision;
    private static double NORMAL_ANGLE_OF_VISION = 180.0;
    private static double MEDIUM_ANGLE_OF_VISION = 90.0;
    private static double SMALL_ANGLE_OF_VISION = 60.0; 
    
    BasicBehaviorGoToBeacon(BasicMovements movements, BasicSensors sensors){
        super(movements, sensors);
        angleOfVision = NORMAL_ANGLE_OF_VISION;
    }

    @Override
    protected boolean TestConitions() {
        if(isBeaconVisible){
            if(     (Math.abs(beaconDir) <= 30.0 && distSensorCenter > BasicSensors.DISTANCE_FAR ) 
                ||  (beaconDir > 30.0 && beaconDir < angleOfVision && distSensorRight > BasicSensors.DISTANCE_FAR ) 
                ||  (beaconDir < -30.0 && beaconDir > (-angleOfVision) && distSensorLeft > BasicSensors.DISTANCE_FAR ) )
                return true;
            else{
                if(beaconDir< SMALL_ANGLE_OF_VISION && beaconDir > -SMALL_ANGLE_OF_VISION)
                    angleOfVision = SMALL_ANGLE_OF_VISION;
            HelperFunctions.print_to_output("Measures: angleOfVision=" +angleOfVision+  "  beaconDir=" +beaconDir+"\n");            
            } 
        }
        return false;
    }

    @Override
    protected void DoBehavior() {
        angleOfVision = NORMAL_ANGLE_OF_VISION;
        
        defaultDirection = (beaconDir > 0) ? BasicMovements.RIGHT : BasicMovements.LEFT;
        if(Math.abs(beaconDir) < 5.0)
            movements.Move(BasicMovements.NORMAL_SPEED);
        else if(Math.abs(beaconDir) < 10.0)
            movements.MoveInTurn(BasicMovements.NORMAL_SPEED, BasicMovements.LIGHT_TURN , defaultDirection);
        else if(Math.abs(beaconDir) < 30.0)
            movements.MoveInTurn(BasicMovements.NORMAL_SPEED, BasicMovements.MEDIUM_TURN , defaultDirection);
        else if(Math.abs(beaconDir) < 80.0)
            movements.MoveInTurn(BasicMovements.NORMAL_SPEED, BasicMovements.HARD_TURN , defaultDirection);
        else
            movements.RotateAroundWheel(BasicMovements.NORMAL_SPEED, defaultDirection);
    }
    
}
