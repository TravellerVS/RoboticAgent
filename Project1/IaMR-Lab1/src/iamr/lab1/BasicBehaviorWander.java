/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

/**
 *
 * @author vedran
 */
public class BasicBehaviorWander extends BasicBehavior {  
    
    protected boolean defaultDirection;
    
    BasicBehaviorWander(BasicMovements movements, BasicSensors sensors){
        super(movements, sensors); 
        defaultDirection = (HelperFunctions.randBool()) ? BasicMovements.RIGHT : BasicMovements.LEFT ;
    }

    @Override
    protected boolean TestConitions() {
        return true;
    }

    @Override
    protected void DoBehavior() {
        //randomizing the direction of movement for the robot
        int maxRange = 100;
        int randNum = HelperFunctions.randInt(0, maxRange);
        if(randNum < maxRange*(0.35)){
            movements.Move(BasicMovements.NORMAL_SPEED);
        }else{
            //this if statement is to change the default direction of the turning for better woundering behaviour
            if(randNum > maxRange*(0.95)){
                //change default direction
                defaultDirection = (defaultDirection == BasicMovements.RIGHT) ? BasicMovements.LEFT: BasicMovements.RIGHT;
            }
            movements.MoveInTurn( BasicMovements.NORMAL_SPEED, BasicMovements.HARD_TURN, defaultDirection );
        }
    }
    
}
