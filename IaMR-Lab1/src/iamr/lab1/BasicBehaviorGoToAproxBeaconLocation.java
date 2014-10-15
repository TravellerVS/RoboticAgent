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
    
    BasicBehaviorGoToAproxBeaconLocation(BasicMovements movements, BasicSensors sensors)
    {
        super(movements, sensors);
    }
    
    @Override
    protected boolean TestConitions() {
        if(isAproxBeaconSet()){
            if(     (Math.abs(aproxBeaconDir) < 10.0 && distSensorCenter > BasicSensors.DISTANCE_FAR ) 
                ||  (aproxBeaconDir > 10.0 && beaconDir <=150 && distSensorRight > BasicSensors.DISTANCE_FAR ) 
                ||  (aproxBeaconDir > -10.0 && beaconDir <=-150 && distSensorLeft > BasicSensors.DISTANCE_FAR ) )
                return true; 
        }
        return false;
    }

    @Override
    protected void DoBehavior() {
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
