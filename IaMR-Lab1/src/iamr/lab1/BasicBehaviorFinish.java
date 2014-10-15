/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

/**
 *
 * @author vedran
 */
public class BasicBehaviorFinish extends BasicBehavior{

    BasicBehaviorFinish(BasicMovements movements, BasicSensors sensors)
    {
        super(movements, sensors);
    }

    @Override
    protected boolean TestConitions() {
        if(ground == GetFinalBeacon())
            return true;    
        return false;
    }

    @Override
    protected void DoBehavior() {
        movements.Stop();
    }
    
}
