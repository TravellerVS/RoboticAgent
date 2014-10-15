/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

/**
 *
 * @author vedran
 */
public class PrioritizedBehavior {
    public int priority;
    public BasicBehavior behavior;
    
    PrioritizedBehavior(BasicBehavior behavior, int priority)
    {
        this.priority = priority;
        this.behavior = behavior;
    }
}
