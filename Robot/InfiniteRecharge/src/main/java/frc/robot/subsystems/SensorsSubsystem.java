package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardControlSystem;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.logging.Logger;

public class SensorsSubsystem extends SubsystemBase{
    private final Logger logger = Logger.getLogger(this.getClass().getName());
    AnalogInput UltraSonic1  = new AnalogInput(3);
    // AnalogPotentiometer UltraSonic1 = new AnalogPotentiometer(3, 1023, 59);  //340
    
   
    public SensorsSubsystem() {

    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Ultrasonic Value", getUltraSonic1() * 0.125);
      // SmartDashboard.putNumber("Ultrasonic Old value", UltraSonic2.getValue());
      // SmartDashboard.putNumber("Ultrasonic getValue System", UltraSonic1.getValue());
      // SmartDashboard.putNumber("Ultrasonic GetVoltage system", UltraSonic1.getVoltage());
    }
    
    public double getUltraSonic1(){
      return UltraSonic1.getValue();
    }
}