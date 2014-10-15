/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

/**
 *
 * @author vedran
 */
public class BasicBehaviorGoToAproxBeaconLocation extends BasicBehavior{

    private boolean defaultDirection;
    private double angleOfVision;
    private static double NORMAL_ANGLE_OF_VISION = 180.0;
    private static double MEDIUM_ANGLE_OF_VISION = 90.0;
    private static double SMALL_ANGLE_OF_VISION = 60.0; 
    
    BasicBehaviorGoToAproxBeaconLocation(BasicMovements movements, BasicSensors sensors){
        super(movements, sensors);
        angleOfVision = NORMAL_ANGLE_OF_VISION;
    }
    
    @Override
    protected boolean TestConitions() {
        if(isAproxBeaconSet()){
            if(     (Math.abs(aproxBeaconDir) < 30.0 && distSensorCenter > BasicSensors.DISTANCE_FAR ) 
                ||  (aproxBeaconDir > 30.0 && aproxBeaconDir < angleOfVision && distSensorRight > BasicSensors.DISTANCE_FAR ) 
                ||  (aproxBeaconDir < -30.0 && aproxBeaconDir > (-angleOfVision) && distSensorLeft > BasicSensors.DISTANCE_FAR ) )
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
        defaultDirection = (aproxBeaconDir > 0) ? BasicMovements.RIGHT : BasicMovements.LEFT;
        
        if(Math.abs(aproxBeaconDir) < 5.0)
            movements.Move(BasicMovements.NORMAL_SPEED);
        else if(Math.abs(aproxBeaconDir) < 10.0)
            movements.MoveInTurn(BasicMovements.NORMAL_SPEED, BasicMovements.LIGHT_TURN , defaultDirection);
        else if(Math.abs(aproxBeaconDir) < 30.0)
            movements.MoveInTurn(BasicMovements.NORMAL_SPEED, BasicMovements.MEDIUM_TURN , defaultDirection);
        else if(Math.abs(aproxBeaconDir) < 80.0)
            movements.MoveInTurn(BasicMovements.NORMAL_SPEED, BasicMovements.HARD_TURN , defaultDirection);
        else
            movements.Rotate(BasicMovements.NORMAL_SPEED, defaultDirection);
    }    
}
