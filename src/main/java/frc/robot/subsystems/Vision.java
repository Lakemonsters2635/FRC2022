package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Vision extends Subsystem {
	
	
	NetworkTableEntry tx;
	NetworkTableEntry ty;
	NetworkTableEntry ta;
	NetworkTableEntry ts;
	NetworkTableEntry tv;
	NetworkTableEntry pipeline;
	double x,y,area,skew,v;
	public NetworkTableEntry camMode;
	NetworkTableEntry ledMode;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public Vision(){
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
		ts = table.getEntry("ts");
		tv = table.getEntry("tv");
		camMode = table.getEntry("camMode");
		ledMode = table.getEntry("ledMode");
		
		pipeline = table.getEntry("pipeline");
		if (pipeline != null)
		{
			//System.out.println("pipeline:" + pipeline) ;
			pipeline.setNumber(0);
		}
		
		
		//Turn off LED
		ledMode.setDouble(1);
	}
	
	public void driveMode(){
		//Turn off LED
		ledOff();
		//Set cam to drive mode
		camMode.setDouble(1);
	}
	
	public boolean isLightOn() {
		boolean isOn = (ledMode.getDouble(0) == 0);
		return isOn;
	}
	public void ledOff(){
		ledMode.setDouble(1);
	}
	
	public void ledOn(){
		ledMode.setDouble(0);
	}

	public double getLed(){
		return ledMode.getDouble(0);
	}
	
	public void data() {
		//Set cam mode to vision processing
		camMode.setDouble(0);
		//Turn on LED
		ledOn();
		//Get x and y angles
		x = tx.getDouble(0);
		y = ty.getDouble(0);
		v = tv.getDouble(0);
		//Get area of target
		area = ta.getDouble(1);
		//Get skew of target
		skew = ts.getDouble(0);
	}
	
	public boolean targetExists() {
		if(v == 1.0)
		{

			double targetDistance = getXDistance();
			// SmartDashboard.putNumber("targetDistance", targetDistance);
			return true;
		}
		return false;
	}
	
	public double getXAngle() {
		// SmartDashboard.putNumber("xAngle", x);
		return x;
	}
	
	public double getYAngle() {
		return y;
	}
	
	public double getXDistance() {
		double angle = getYAngle();
		// SmartDashboard.putNumber("y angle", angle);
		//double xDistance = -30.6 * angle - 254;
		double xDistance = -18.7 * angle - 80.7;
		return xDistance;
		//return RobotMap.TARGET_HEIGHT/Math.tan(Math.toRadians(angle)+RobotMap.CAMERA_ANGLE);
	}

	public double getArea() {
		//System.out.println(area);
		return area;
	}
	public double printXAngle() {
		double angle = getXAngle();
		//SmartDashboard.putNumber("Xangle:", angle);
		return angle;
	}
	public void printArea() {
		//SmartDashboard.putNumber("Area:", getArea());
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}