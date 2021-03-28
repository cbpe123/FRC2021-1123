package frc.robot.subsystems;

import java.sql.Time;
import java.util.logging.Logger;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

import frc.robot.RobotContainer;
import frc.robot.DashboardControlSystem;
import frc.robot.JoystickControlSystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;


/**
 * Creates a new ShooterSubsystem.
 */
public class ClimberSubsystem extends SubsystemBase {
  private Logger logger = Logger.getLogger(this.getClass().getName());
  private CANSparkMax motorA = new CANSparkMax(Constants.ClimberMotorLeftCanID, MotorType.kBrushless);
  private CANSparkMax motorB = new CANSparkMax(Constants.ClimberMotorRightCanID, MotorType.kBrushless);
  private DoubleSolenoid ClimbSolenoid = new DoubleSolenoid(Constants.ClimberPCM,
      Constants.ClimberSolenoidForward, Constants.ClimberSolenoidReverse);
  private double motorSetPoint = 0.5;
  private boolean subsystemActive = false;

  int Time = 0;
  double StartTime = 0;
  CANEncoder motorAEncoder = motorA.getEncoder();
  CANEncoder motorBEncoder = motorA.getEncoder();
  // Timer GameTimer = new Timer();

  public ClimberSubsystem() {
    // Wipe any prior motor settings

    // Set motor direction

    // TODO: Set motor current limits

    ClimbSolenoid.set(Value.kForward);
  }

  public void SpinMotorTogetherUp() {
    OpenWinch();
    if(Time > 40){
      motorA.set(motorSetPoint);
      motorB.set(motorSetPoint);
      logger.info("Both going up");
    }
    subsystemActive = true;

    logger.info("Climber trying to spin at " + motorSetPoint);
    logger.info("Both up");
    // SmartDashboard.putNumber("Climber Motor 1 RPM ", motorAEncoder.getVelocity());
    // SmartDashboard.putNumber("Climber Motor 2 RPM ", motorBEncoder.getVelocity());
  }

  public void SpinMotorTogetherDown() {
    motorA.set(-motorSetPoint);
    motorB.set(-motorSetPoint);


    subsystemActive = true;

    logger.info("Climber trying to spin at " + -motorSetPoint);
    logger.info("Both down");
    // SmartDashboard.putNumber("Climber Motor 1 RPM ", motorAEncoder.getVelocity());
    // SmartDashboard.putNumber("Climber Motor 2 RPM ", motorBEncoder.getVelocity());
  }

  public void SpinMotorTogetherDownSlow() {
    motorA.set(-motorSetPoint/2);
    motorB.set(-motorSetPoint/2);


    subsystemActive = true;

    logger.info("Climber trying to spin at " + -motorSetPoint/2);
    // SmartDashboard.putNumber("Climber Motor 1 RPM ", motorAEncoder.getVelocity());
    // SmartDashboard.putNumber("Climber Motor 2 RPM ", motorBEncoder.getVelocity());
  }

  public void SpinMotorAUp(){
    motorA.set(motorSetPoint);
  }

  public void SpinMotorADown(){
    OpenWinch();
    motorA.set(-motorSetPoint);
   
  }

  public void SpinMotorADownSlow(){
    motorA.set(-motorSetPoint/2);
  }

  public void StopMotorA(){
    motorA.set(0);
  }

  public void SpinMotorBUp(){
    OpenWinch();
    motorB.set(motorSetPoint);
  }

  public void SpinMotorBDown(){
    motorB.set(-motorSetPoint);
  }

  public void SpinMotorBDownSlow(){
    motorB.set(-motorSetPoint/2);
  }

  public void StopMotorB(){
    motorB.set(0);
  }

  public void OpenWinch(){
    // if(Timer.getFPGATimestamp()-StartTime > 89){
      ClimbSolenoid.set(Value.kForward);
      logger.info("winch opened");
    // }
  }  
  
  public void CloseWinch(){
    ClimbSolenoid.set(Value.kReverse);
    logger.info("winch closed");
  }

  public void Stop() {
    motorA.set(0);
    motorB.set(0);
    Time = 0;
    subsystemActive = false;
  }

  @Override
  public void periodic() {
    // TODO: Update dashboard motor speed via NetworkTables
    SmartDashboard.putNumber("Climber Motor 1 RPM ", motorAEncoder.getVelocity());
    SmartDashboard.putNumber("Climber Motor 2 RPM ", motorBEncoder.getVelocity());
    // if(Timer.getFPGATimestamp()-StartTime>=118){
    //   CloseWinch();
    // }
    motorSetPoint = JoystickControlSystem.getClimberThrottle()*0.7;
    SmartDashboard.putNumber("Winch Set Point", motorSetPoint);
    
    int intTime = (int)(135.0-(Timer.getFPGATimestamp()-StartTime));
    // SmartDashboard.putNumber("Time Left", intTime);
    DashboardControlSystem.putTimeRemaining(intTime);
    // logger.info("In Climber subsystem periodic");
    Time++;
  }

  public boolean isActive(){
    return this.subsystemActive;
  }

  public double getSetSpeed(){
    return motorSetPoint;
  }

  public void setSpeed(double desiredSpeed){
    this.motorSetPoint = desiredSpeed;
  }

  public void setStartTime(double time){
    StartTime = time;
  }

  public void resetTime(){
    Time = 0;
  }
}