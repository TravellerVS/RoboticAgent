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
    
    private double x1, y1, dir1, beaconDir1, compas1;
    
    protected double aproxBeaconX, aproxBeaconY, aproxBeaconDir, aproxBeaconAngleDifference = 0.0;
    protected double GetAproxBeaconDir(){
        return aproxBeaconDir;  
    }
    protected boolean isAproxBeaconSet(){
        return (aproxBeaconAngleDifference != 0.0);  
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
            if(x1==0.0 && y1== 0.0 && compas1 == 0.0 && x!=0.0 && y!= 0.0 && compas != 0.0){
                x1 = x;
                y1= y;
                compas1 = compas;
                beaconDir1 = beaconDir;
                //HelperFunctions.print_to_output("------------point1 set-------------------------\n");
            }else{
                //HelperFunctions.print_to_output("------------point1 set-------------------------\n");
                //HelperFunctions.print_to_output("------------distance = " + DistanceBetweenPoints(x1,y1,x, y) + "-------------------------\n");
                if(DistanceBetweenPoints(x1,y1,x, y) > 5.0){
                    SetAproxBeaconLocation(x1,y1,compas1,beaconDir1, x, y, compas, beaconDir);
                }
            }            
        }
        
        if(aproxBeaconAngleDifference != 0.0)
        {
            aproxBeaconDir = NormalizeAngle( compas - GetAngleBetweenPoints(x, y, aproxBeaconX, aproxBeaconY));
            HelperFunctions.print_to_output("Measures: AproxBeaconDir=" +aproxBeaconDir + "aproxBeaconX=" +aproxBeaconX + "aproxBeaconY=" +aproxBeaconY + "\n");
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
        return Math.toDegrees(Math.asin((y2-y1)/(x2-x1)));
    }
    
    private double NormalizeAngle(double angle) {
        if(angle > 180.00){
            return (-360.0 + angle);
        }else if(angle < -180.00){
            return (3600.0 + angle);
        }else{
            return angle;
        }
    }
}
