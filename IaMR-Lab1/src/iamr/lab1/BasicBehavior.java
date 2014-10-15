/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

/**
 *
 * @author vedran
 */
public abstract class BasicBehavior {
    
    public int priority_group;
    public int priority;
    
    protected BasicMovements movements;
    protected BasicSensors sensors;
    
    protected double distSensorLeft, distSensorRight, distSensorCenter, beaconDir, compas;
    protected boolean isBeaconVisible;
    
    protected int    ground;
    protected boolean collision;
    protected double x,y,dir;

    protected int beaconToFollow;    
    protected int GetFinalBeacon(){return beaconToFollow;}
    
    protected static int HOME_GROUND = 1;
    protected static int UNKNOWN_GROUND = -1;
    
    private boolean sensorReadingsFetched = false;
        
    protected double aproxBeaconDir;
    protected double GetAproxBeaconDir(){
        return sensors.GetAproxBeaconDir();  
    }
    protected boolean isAproxBeaconSet(){
        return sensors.isAproxBeaconSet();  
    }
    
    
    public BasicBehavior(BasicMovements movements, BasicSensors sensors)
    {
        this.movements = movements;
        this.sensors = sensors;
    }
    
    public boolean isExecutable()
    {
        FetchSensorReadings();
        return TestConitions();
    };
    
    public void Execute()
    {
        BehaviorStarted();
        DoBehavior();
        BehaviorExecuted();        
    };
    
    protected abstract boolean TestConitions();
    protected abstract void DoBehavior();
    
    public void Reset(){
        sensorReadingsFetched = false;
    }
    
    protected void FetchSensorReadings() {
        if(SensorReadingsFetched())
            return;
        distSensorCenter = sensors.GetIRSensor0();
        distSensorLeft = sensors.GetIRSensor1();
        distSensorRight = sensors.GetIRSensor2();        
        ground = sensors.GetGround();
        beaconToFollow = sensors.GetBeaconToFollow();
        x = sensors.GetX();
        y = sensors.GetY();
        dir = sensors.GetDir();
        compas = sensors.GetCompas();
        collision = sensors.GetCollision();
        if(sensors.isBeaconVisible())
        {
            isBeaconVisible = true;
            beaconDir = sensors.GetBeaconDir();
                     
        }
        if(isAproxBeaconSet())
        {
            aproxBeaconDir = GetAproxBeaconDir();
        }
        sensorReadingsFetched = true;
    }
    protected boolean SensorReadingsFetched() {
        return sensorReadingsFetched;
    }
    protected void BehaviorStarted() {
        if(!SensorReadingsFetched()){
            FetchSensorReadings();
        }
    }
    protected void BehaviorExecuted() {
        sensorReadingsFetched = false;
    }
}
