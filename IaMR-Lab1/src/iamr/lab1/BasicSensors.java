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
    
    public static double DISTANCE_ALMOST_COLIDED = -4.5;
    public static double DISTANCE_VERY_CLOSE = -4.0;
    public static double DISTANCE_IDEAL = -3.9;
    public static double DISTANCE_CLOSE = -3.0;
    public static double DISTANCE_NEAR = -2.8;
    public static double DISTANCE_MEDIUM = -1.5;
    public static double DISTANCE_FAR = -1.0;
    public static double DISTANCE_VISIBLE = 0.0;
    
    private double irSensor0, irSensor1, irSensor2, compas;
    public double GetIRSensor0(){return -irSensor0;}
    public double GetIRSensor1(){return -irSensor1;}
    public double GetIRSensor2(){return -irSensor2;}
    public double GetCompas(){return compas;}
    
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
    
    private double x1, y1, dir1, beaconDir1, compas1;
    
    protected double aproxBeaconX, aproxBeaconY, aproxBeaconDir, aproxBeaconAngleDifference = 0.0;
    protected double GetAproxBeaconDir(){
        return aproxBeaconDir;  
    }
    protected boolean isAproxBeaconSet(){
        return (aproxBeaconAngleDifference != 0.0);  
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
                compas = cif.GetCompassSensor();
        if(cif.IsGroundReady())
                ground = cif.GetGroundSensor();
        if(cif.IsBumperReady())
                collision = cif.GetBumperSensor();
        if(cif.IsBeaconReady(beaconToFollow))
        {
            beacon = cif.GetBeaconSensor(beaconToFollow);
            if(isBeaconVisible())
            {
                if(x1==0.0 && y1== 0.0 && compas1 == 0.0 && x!=0.0 && y!= 0.0 && compas != 0.0){
                    x1 = x;
                    y1= y;
                    compas1 = compas;
                    beaconDir1 = GetBeaconDir();
                    //HelperFunctions.print_to_output("------------point1 set-------------------------\n");
                }else{
                    //HelperFunctions.print_to_output("------------point1 set-------------------------\n");
                    //HelperFunctions.print_to_output("------------distance = " + DistanceBetweenPoints(x1,y1,x, y) + "-------------------------\n");
                    if(DistanceBetweenPoints(x1,y1,x, y) > 5.0){
                        SetAproxBeaconLocation(x1,y1,compas1,beaconDir1, x, y, compas, GetBeaconDir());
                    }
                }            
            }
        }
        
        //SetAproxBeaconLocation(0.0,0.0,0.0,-45.0, 5.0, 0.0, 0.0, -90.0);
        
        if(aproxBeaconAngleDifference != 0.0)
        {
            aproxBeaconDir = NormalizeAngle( GetAngleBetweenPoints(x, y, aproxBeaconX, aproxBeaconY)-compas);
            //aproxBeaconDir = NormalizeAngle( 0.0 - GetAngleBetweenPoints(0.0, 0.0, aproxBeaconX, aproxBeaconY));
            HelperFunctions.print_to_output("Measures: AproxBeaconDir=" +aproxBeaconDir + "aproxBeaconX=" +aproxBeaconX + "aproxBeaconY=" +aproxBeaconY + "\n");
        }
        
        HelperFunctions.print_to_output("------------point1 set-------------------------\n");
        //HelperFunctions.print_to_output("------------distance = " + DistanceBetweenPoints(x1,y1,x, y) + "--- angle:"+GetAngleBetweenPoints(0.0, 0.0, aproxBeaconX, aproxBeaconY)+"\n");
        
        x = cif.GetX();
        y = cif.GetY();
        dir = cif.GetDir();
        
//        HelperFunctions.print_to_output("Measures: ir0=" + irSensor0 + " ir1=" + irSensor1 + " ir2=" + irSensor2 + "\n");
//        HelperFunctions.print_to_output("Measures: compas=" + compas + " ground=" + ground + " isBeaconVisible=" + isBeaconVisible() + " GetBeaconDir=" + GetBeaconDir() + " beaconToFollow=" + beaconToFollow +"\n");
        //HelperFunctions.print_to_output("Measures: x=" + x + " y=" + y + " dir=" + dir);
        
        
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
    
    private double DistanceBetweenPoints(double x1, double y1, double x2, double y2) {        
        return Math.sqrt( Math.pow((x2-x1), 2.0) + Math.pow((y2-y1), 2.0));        
    }
    
    private void SetAproxBeaconLocation(double x1, double y1, double compas1, double beaconDir1, double x2, double y2, double compas2, double beaconDir2) {       
        double tempAngleDifference = Math.abs(compas1 + beaconDir1) - (compas2 + beaconDir2);
        if(Math.abs(90.00 - aproxBeaconAngleDifference) < Math.abs(90.00 - tempAngleDifference))
            return;
        
        aproxBeaconAngleDifference = tempAngleDifference;
        
        double aproxBeaconDir1 = Math.toRadians(compas1 + beaconDir1);
        double aproxBeaconDir2 = Math.toRadians(180.0 - compas2 + beaconDir2);
        
        double distance12 = DistanceBetweenPoints(x1, y1, x2, y2);
        double distance2 = (distance12 - Math.tan(aproxBeaconDir1)/Math.tan(aproxBeaconDir2)) / (1.0 + Math.tan(aproxBeaconDir1)/Math.tan(aproxBeaconDir2));
        
        //aproxBeaconDir2 = Math.toRadians(compas2 + beaconDir2);
        
        aproxBeaconX = x2 - distance2*Math.cos(aproxBeaconDir2);
        aproxBeaconY = y2 + distance2*Math.sin(aproxBeaconDir2);
    }
    
    private double GetAngleBetweenPoints(double x1, double y1, double x2, double y2) { 
        return Math.toDegrees(Math.atan2((y2-y1),(x2-x1)));
    }
    
    private double NormalizeAngle(double angle) {
        if(angle > 180.00){
            return (-360.0 + angle);
        }else if(angle < -180.00){
            return (360.0 + angle);
        }else{
            return angle;
        }
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
