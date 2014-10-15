/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

import ciberIF.*;
import java.util.*;
import java.util.Map.*;

/**
 *This class contains the implementation of the 
 * @author vedran
 */
public class BasicAgent extends Agent implements Observer{
 
    private String _name;
    private void SetName(String name){ this._name = name; }
    public String GetName(){ return this._name; }
    
    private int _pos;
    private void SetPos(int pos){ this._pos = pos; }
    public int GetPos(){ return this._pos; }
    
    private String _host;    
    private void SetHost(String host){ this._host = host; }
    public String GetHost(){ return this._host; }
    
    private BasicIF myIF;
    private BasicMovements movements;
    private BasicSensors sensors;
    
    private PrioritizedBehaviorList behaviorList;
    
    BasicAgent(String robName, int pos, String host) {
        this.SetName(robName);
        this.SetPos(pos);
        this.SetHost(host);
        ciberIF cif = new ciberIF();
        myIF = new BasicIF(cif);
        movements = new BasicMovements(cif);
        sensors = new BasicSensors(cif);
        this.setSubject(sensors);
        
        this.SetUpBehaviors();
    }

    @Override
    void Init() {
        this.myIF.Init(this.GetName(), this.GetPos(),this.GetHost());
    }

    @Override
    void Start() {
        BasicBehavior behavior;
        while(true) {
            this.sensors.ReadSensors();            
            this.sensors.Say(this.GetName());
            
            //movements.Rotate(0.1);
            behavior  = this.behaviorList.ExecuteBehavior();
            
            if(!(behavior instanceof BasicBehavior))
                continue;
                
            HelperFunctions.print_to_output( "Behavior(" + this.GetName() + "): " + behavior.getClass() );
            
            if(behavior instanceof BasicBehaviorFinish)
            {
                HelperFunctions.print_to_output(this.GetName() + " found home at " + myIF.GetTime() + "\n");
                
                if(myIF.GetFinished())
                    System.exit(0);
                else
                    myIF.Finish();
            }
	}
    }

    @Override
    public void Update() {
        Map<String, String> messages = this.sensors.GetMessages();
        for(Entry<String, String> message : messages.entrySet()){
            HelperFunctions.print_to_output("Message: From " + message.getKey() + " to " + this.GetName() + " : \"" + message.getValue() + "\"");
        }
    }

    @Override
    public void setSubject(Subject sub) {
        sub.Register(this);
    }    
    
    private void SetUpBehaviors() {        
        behaviorList = new PrioritizedBehaviorList();
        int i = 100;
        behaviorList.AddBehavior(new BasicBehaviorAvoidColision(movements, sensors) , i--);
        behaviorList.AddBehavior(new BasicBehaviorFinish(movements, sensors) , i--);
        behaviorList.AddBehavior(new BasicBehaviorGoToBeacon(movements, sensors) , i--);
        behaviorList.AddBehavior(new BasicBehaviorGoToAproxBeaconLocation(movements, sensors) , i--);
        behaviorList.AddBehavior(new BasicBehaviorFollowWall(movements, sensors) , i--);        
        behaviorList.AddBehavior(new BasicBehaviorWander(movements, sensors) , i--);
    }
}
