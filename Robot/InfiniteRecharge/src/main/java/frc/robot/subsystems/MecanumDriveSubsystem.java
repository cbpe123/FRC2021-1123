package frc.robot.subsystems;

import edu.wpi.first.wpilibj.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.AdvancedMecanumDrive;
import java.lang.Math;

// import java.util.Timer;
import java.util.logging.Logger;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

// import org.graalvm.compiler.lir.aarch64.AArch64Move.LoadAddressOp;

import frc.robot.Constants;
import frc.robot.DashBoardHelper;
import frc.robot.RobotContainer;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;

public class MecanumDriveSubsystem extends SubsystemBase {
  private final Logger logger = Logger.getLogger(this.getClass().getName());
  private AdvancedMecanumDrive m_robotDrive;
  WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.kFrontLeftChannel);
  WPI_TalonFX rearLeft = new WPI_TalonFX(Constants.kRearLeftChannel);
  WPI_TalonFX frontRight = new WPI_TalonFX(Constants.kFrontRightChannel);
  WPI_TalonFX rearRight = new WPI_TalonFX(Constants.kRearRightChannel);

  double XDistanceTraveled = 0;
  double YDistanceTraveled = 0;
  double magnitude = 0.2;
  double setX = 0;
  double setY = 0;
  double angle = 0;
  double AverageWheelSpeed = 0;
  int hertzCounter = 0;
  DashBoardHelper yDistanceTraveledDashBoard = new DashBoardHelper();
  DashBoardHelper xDistanceTraveledDashBoard = new DashBoardHelper();
  DashBoardHelper angleDashBoard = new DashBoardHelper();
  

  /**
   * Creates a new MecanumDriveSubsystem.
   */
  public MecanumDriveSubsystem() {
    WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.kFrontLeftChannel);
    WPI_TalonFX rearLeft = new WPI_TalonFX(Constants.kRearLeftChannel);
    WPI_TalonFX frontRight = new WPI_TalonFX(Constants.kFrontRightChannel);
    WPI_TalonFX rearRight = new WPI_TalonFX(Constants.kRearRightChannel);

    frontLeft.setNeutralMode(NeutralMode.Coast);
    rearLeft.setNeutralMode(NeutralMode.Coast);
    frontLeft.setNeutralMode(NeutralMode.Coast);
    rearRight.setNeutralMode(NeutralMode.Coast);
    
    frontLeft.configOpenloopRamp(0.0);
    frontRight.configOpenloopRamp(0.0);
    rearLeft.configOpenloopRamp(0.0);
    rearRight.configOpenloopRamp(0.0);

    m_robotDrive = new AdvancedMecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    logger.info("The mecanum drive subsystem is initialized.");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: Implement periodic functionality such as updating dashboards/logs
    yDistanceTraveledDashBoard.updateValue("YDistance Travelled", YDistanceTraveled, 0.5);
    // xDistanceTraveledDashBoard.updateValue("XDistance Travelled", XDistanceTraveled, 0.5);
  }

  public void driveCartesian(double yval, double xval, double zval, double throttle) {
    //logger.info(String.format("X: %s, Y: %s, Z: %s, G: %s", xval, yval, zval, gyroval));
    m_robotDrive.driveCartesian(-yval, -xval, zval, throttle);
  }

  public void FieldCartesian(double yval, double xval, double zval, double throttle, double angle) {
    //logger.info(String.format("X: %s, Y: %s, Z: %s, G: %s", xval, yval, zval, gyroval));
    m_robotDrive.driveCartesian(-yval, -xval, zval, throttle, angle);
  }

  public void pivotCartesian(double yval, double xval, double zval, double throttle) {
    //logger.info(String.format("X: %s, Y: %s, Z: %s, G: %s", xval, yval, zval, gyroval));
    m_robotDrive.pivotCartesian(-yval, -xval, zval, throttle);
  }

  public void drivePolar(double magnitude, double angle, double zRotation){
    m_robotDrive.drivePolar(magnitude, angle, zRotation);
  }

  public void setMagnitude(double setMagnitude){
    magnitude = setMagnitude;
  }

  public void setDrivePositionMode(){
  //   frontLeft.setNeutralMode(NeutralMode.Coast);
  //   rearLeft.setNeutralMode(NeutralMode.Coast);
  //   frontLeft.setNeutralMode(NeutralMode.Coast);
  //   rearRight.setNeutralMode(NeutralMode.Coast);
    
    frontLeft.configOpenloopRamp(0.2);
    frontRight.configOpenloopRamp(0.2);
    rearLeft.configOpenloopRamp(0.2);
    rearRight.configOpenloopRamp(0.2);
  }
  
  public void turnOffDrivePositionMode(){
    // frontLeft.setNeutralMode(NeutralMode.Brake);
    // rearLeft.setNeutralMode(NeutralMode.Brake);
    // frontLeft.setNeutralMode(NeutralMode.Brake);
    // rearRight.setNeutralMode(NeutralMode.Brake);
    
    frontLeft.configOpenloopRamp(0.0);
    frontRight.configOpenloopRamp(0.0);
    rearLeft.configOpenloopRamp(0.0);
    rearRight.configOpenloopRamp(0.0);
  }

  public void setAngleFromXY(double x, double y){
    setX = x;
    setY = y;
    if(x == 0){
        if(y > 0)
            angle = 90;
        else
            angle = 270;
    }
    else if(y == 0){
        if(x > 0)
            angle = 0;
        else
            angle = 180;
    }
    else if(x >= 0)
        angle = Math.toDegrees(Math.atan(y/x));
    else
        angle = 180 + Math.toDegrees(Math.atan(y/x));
    angle = angle - 90;
    if(angle > 180)
        angle = angle - 360;

    SmartDashboard.putNumber("Angle from set position", angle);
  }

  public void drivePosition(){
    m_robotDrive.drivePolar(magnitude, angle, 0);

    double localAngle = angle + 90;// because of drivePolar
    double ticksPerTenthSecond = (Math.abs(frontLeft.getSelectedSensorVelocity()) + Math.abs(frontRight.getSelectedSensorVelocity())
    + Math.abs(rearLeft.getSelectedSensorVelocity()) + Math.abs(rearRight.getSelectedSensorVelocity()))/4;  
    // turnsPerSecond
    double motorTurnsPerSecond = 10 * ticksPerTenthSecond/2048; // 2048 ticks per turn
    double wheelAxleTurnsPerSecond = motorTurnsPerSecond /10; //10 to 1 gearbox
    double inchesPerSecond = wheelAxleTurnsPerSecond * 18.85; // inches per turn
    SmartDashboard.putNumber("AverageWheelSpeed", AverageWheelSpeed);
    if(localAngle == 0 || localAngle == 180){  // sideways
      inchesPerSecond = inchesPerSecond / 5;
    }

    else if(localAngle == 90 || localAngle == 270){ // forwards or backwards
      inchesPerSecond = inchesPerSecond /1;
    }
    
    else{ // 45 degree vectors
      inchesPerSecond = inchesPerSecond * 0.707;
    }
    YDistanceTraveled += inchesPerSecond * Math.toDegrees(Math.sin(localAngle)) / 50; //code runs every 20 ms = 50 Hertz
    XDistanceTraveled += inchesPerSecond * Math.toDegrees(Math.cos(localAngle)) / 50;
    hertzCounter++;
    // if(hertzCounter>=25){
    //   SmartDashboard.putNumber("YDistanceTravelled", YDistanceTraveled);
    //   SmartDashboard.putNumber("XDistanceTravelled", XDistanceTraveled);
    //   hertzCounter = 0;
    // }
    // yDistanceTraveledDashBoard.updateValue("YDistance Travelled", YDistanceTraveled, 0.5);

  }
  

  public boolean atPoint(){
    if(Math.abs(XDistanceTraveled) >= Math.abs(setX) && (Math.abs(YDistanceTraveled) >= Math.abs(setY))){
      return true;
    }
    return false;
  }

  public void ResetDistanceTraveled(){
    XDistanceTraveled = 0;
    YDistanceTraveled = 0;
  }

  //Not a good name this is for moving forward a set amount using encoder posistion
  //I shouldnt be using this will delete later.
  public void driveRotation(double setpoint){
    frontLeft.config_kP(0, 0.1, 30);
    frontRight.config_kP(0, 0.1, 30);
    rearLeft.config_kP(0, 0.1, 30);
    rearRight.config_kP(0, 0.1, 30);

    frontLeft.set(ControlMode.Position, setpoint);
    frontRight.set(ControlMode.Position, setpoint);
    rearLeft.set(ControlMode.Position, setpoint);
    rearRight.set(ControlMode.Position, setpoint);
  }

  private Translation2d FrontLeftTranslation2d = new Translation2d(0.3048,0.2667);
  private Translation2d FrontRightTranslation2d = new Translation2d(0.3048,-0.2667);
  private Translation2d BackLeftTranslation2d = new Translation2d(-0.3048,0.2667);
  private Translation2d BackRightTranslation2d = new Translation2d(-0.3048,-0.2667);

  private  Rotation2d Gyro = new Rotation2d(0);

  // private MecanumDriveKinematics kinetics = new MecanumDriveKinematics(FrontLeftTranslation2d, FrontRightTranslation2d,
  //      BackLeftTranslation2d, BackRightTranslation2d);
  // private MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinetics, Gyro);
  Timer Time = new Timer();
  // private MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(Gyro, new Pose2d(), kinetics,
  //  VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), VecBuilder.fill(Units.degreesToRadians(0.01)),
  //  VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        applyDeadband(frontLeft.getSelectedSensorVelocity(), 1)/2048*4.788/10,
        applyDeadband(frontRight.getSelectedSensorVelocity(), 1)/2048*4.788/10,
        applyDeadband(rearLeft.getSelectedSensorVelocity(), 1)/2048*4.788/10,
        applyDeadband(rearRight.getSelectedSensorPosition(), 1)/2048*4.788/10);
  }

  public void updateOdometry(){
    // Gyro = new Rotation2d(RobotContainer.getInstance().Gyro.getAngle());
    // odometry.updateWithTime(Timer.getFPGATimestamp(), Gyro, getCurrentState());
    // poseEstimator.update(Gyro, getCurrentState());
  }
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      return value;
    } else {
      return 0.0;
    }
  }
}
