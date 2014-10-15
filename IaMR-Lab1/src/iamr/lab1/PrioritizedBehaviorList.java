/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 *
 * @author vedran
 */
public class PrioritizedBehaviorList {
    private List<PrioritizedBehavior> behaviorList;
    
    PrioritizedBehaviorList()
    {
        behaviorList = new ArrayList<PrioritizedBehavior>();
    }
    
    public void AddBehavior(BasicBehavior behavior, int priority)
    {
        behaviorList.add(new PrioritizedBehavior(behavior, priority));
        OrderList();
        return;
    }
    
    private void OrderList()
    {
        Collections.sort(behaviorList, new Comparator<PrioritizedBehavior>(){
            @Override
            public int compare(PrioritizedBehavior o1, PrioritizedBehavior o2) {
                return o2.priority - o1.priority;
            }
        });
    }
    
    public BasicBehavior ExecuteBehavior()
    {
        for(PrioritizedBehavior obj : behaviorList)
        {
//            HelperFunctions.print_to_output(""+obj.behavior.getClass());
            if(obj.behavior.isExecutable()){
                obj.behavior.Execute();
                return obj.behavior;
            }else{
                obj.behavior.Reset();
            }
        }
        return null;
    }
    
}




