/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

import ciberIF.*;
import java.util.*;

/**
 *Class contains interactons with the sensors and methods 
 * which provide readings from the robot
 * @author vedran
 */
public class BasicSensors implements Subject {
    private ciberIF cif;
    
    public static double DISTANCE_VERY_CLOSE = -4.0;
    public static double DISTANCE_NEAR = -2.3;
    public static double DISTANCE_FAR = -1.0;
    
    private double irSensor0, irSensor1, irSensor2, compass;
    public double GetIRSensor0(){return -irSensor0;}
    public double GetIRSensor1(){return -irSensor1;}
    public double GetIRSensor2(){return -irSensor2;}
    public double GetCompas(){return compass;}
    
    private beaconMeasure beacon;
    public double GetBeaconDir(){return -beacon.beaconDir;}
    public boolean isBeaconVisible(){return beacon.beaconVisible;}
    
    private int    ground;
    public int GetGround(){return ground;}
    private boolean collision;
    public boolean GetCollision(){return collision;}
    
    private double x,y,dir;
    public double GetX(){return x;}
    public double GetY(){return y;}
    public double GetDir(){return dir;}
    
    private int beaconToFollow;
    public int GetBeaconToFollow(){return beaconToFollow;}
    public void SetBeaconToFollow(int newBeaconNum){beaconToFollow = newBeaconNum;}
    
    private Map<String, String> messages;
    public Map<String, String> GetMessages()    {       
        return messages;
    }
    /**
     *@brief Constructor mothod
     * 
     *@param cif
     */    
    BasicSensors(ciberIF cif) {
        this.cif = cif;
        this.beacon = new beaconMeasure();
        beaconToFollow = 0;
        ground=-1;
    }
    
    public void ReadSensors()
    {
        this.cif.ReadSensors();

        if(cif.IsObstacleReady(0))
                irSensor0 = cif.GetObstacleSensor(0);
        if(cif.IsObstacleReady(1))
                irSensor1 = cif.GetObstacleSensor(1);
        if(cif.IsObstacleReady(2))
                irSensor2 = cif.GetObstacleSensor(2);
        if(cif.IsCompassReady())
                compass = cif.GetCompassSensor();
        if(cif.IsGroundReady())
                ground = cif.GetGroundSensor();
        if(cif.IsBeaconReady(beaconToFollow))
                beacon = cif.GetBeaconSensor(beaconToFollow);

        x = cif.GetX();
        y = cif.GetY();
        dir = cif.GetDir();
        
//        HelperFunctions.print_to_output("Measures: ir0=" + irSensor0 + " ir1=" + irSensor1 + " ir2=" + irSensor2 + "\n");
//        HelperFunctions.print_to_output("Measures: compass=" + compass + " ground=" + ground + " isBeaconVisible=" + isBeaconVisible() + " GetBeaconDir=" + GetBeaconDir() + " beaconToFollow=" + beaconToFollow +"\n");
        HelperFunctions.print_to_output("Measures: x=" + x + " y=" + y + " dir=" + dir);
        
        
        if(cif.GetTime() % 2 == 0) {
             cif.RequestIRSensor(0);
             if(cif.GetTime() % 8 == 0)
                 cif.RequestGroundSensor();
             else
                 cif.RequestBeaconSensor(beaconToFollow);
        }
        else{
            cif.RequestIRSensor(1);
            cif.RequestIRSensor(2);
        }
        
        FetchMessages();
    }
    
    private void FetchMessages()
    {
        messages = new HashMap<String, String>();
        for(int i=1; i<6; i++)
              if(cif.NewMessageFrom(i))
                  messages.put(Integer.toString(i), cif.GetMessageFrom(i));
        if(!messages.isEmpty())
        {
            NotifyObservers();
        }
    }
    
    public void Say(String message)
    {
        cif.Say(message);
    }
    
    private List<Observer> observers = new ArrayList();
    @Override
    public void Register(Observer obj) {
        this.observers.add(obj);
    }

    @Override
    public void Unregister(Observer obj) {
        if(this.observers.contains(obj))
            this.observers.remove(obj);
    }

    @Override
    public void NotifyObservers() {
        for(Observer observer : this.observers)
            observer.Update();
    }

//    @Override
//    public Object GetUpdate(Observer obj) {
//        throw new UnsupportedOperationException("Not supported yet.");
//    }
}
