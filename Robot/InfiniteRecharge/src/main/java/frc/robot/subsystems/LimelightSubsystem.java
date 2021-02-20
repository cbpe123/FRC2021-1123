package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardControlSystem;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.logging.Logger;

public class LimelightSubsystem extends SubsystemBase{
    private final Logger logger = Logger.getLogger(this.getClass().getName());
    //Distance measuring constants in inches or degrees
    //Height of Limelight
    double H1 = 24.25;
    //Height of Goal to center of bounding box
    //This is estimation test to see where it actually is
    double  H2 = 65;
    //This is angle of limelight to horizontal
    double a1 = 0;
   
    public LimelightSubsystem() {
        
    }
  
    public void SwitchLedState() {
          logger.info("Changing LED Status");
          Number ledState = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(1);
          logger.info("Current LED state" + ledState.intValue());
  
          if (ledState.intValue() == 1) {
                  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
          }
          else if (ledState.intValue() == 3) {
                  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
          }
    }
  
    public void DisplayValues() {
      Double xval = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
      Double yval = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty0").getDouble(0.0);
      Double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta0").getDouble(0.0);
  
      SmartDashboard.putNumber("Xval", xval);
      SmartDashboard.putNumber("Yval", yval);
      SmartDashboard.putNumber("Area", area);
      }
  
    public static void getArea() { // Not Really Necessary
      Double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
      SmartDashboard.putNumber("Area", area);
    }

    public double getX(){
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(.0);
    }
    
    public boolean ifBox(){
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
    }
    @Override
    public void periodic() {
      DashboardControlSystem.PutIsBox(ifBox());
      getArea();
      SmartDashboard.putNumber("Limelight X Val for reals", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(.0));
      SmartDashboard.putNumber("Distance To Target Test", getDistance());
    }

    public double getDistance(){ // untested and done with experimental values
      double distance = (H2-H1)/Math.tan(a1 + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(.0));
      return distance;
    }
}