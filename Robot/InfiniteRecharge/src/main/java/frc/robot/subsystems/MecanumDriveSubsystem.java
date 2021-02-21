package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.AdvancedMecanumDrive;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MecanumDriveSubsystem extends SubsystemBase {
  private final Logger logger = Logger.getLogger(this.getClass().getName());
  private AdvancedMecanumDrive m_robotDrive;
  WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.kFrontLeftChannel);
  WPI_TalonFX rearLeft = new WPI_TalonFX(Constants.kRearLeftChannel);
  WPI_TalonFX frontRight = new WPI_TalonFX(Constants.kFrontRightChannel);
  WPI_TalonFX rearRight = new WPI_TalonFX(Constants.kRearRightChannel);

  /**
   * Creates a new MecanumDriveSubsystem.
   */
  public MecanumDriveSubsystem() {
    WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.kFrontLeftChannel);
    WPI_TalonFX rearLeft = new WPI_TalonFX(Constants.kRearLeftChannel);
    WPI_TalonFX frontRight = new WPI_TalonFX(Constants.kFrontRightChannel);
    WPI_TalonFX rearRight = new WPI_TalonFX(Constants.kRearRightChannel);

    frontLeft.setNeutralMode(NeutralMode.Brake);
    rearLeft.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    rearRight.setNeutralMode(NeutralMode.Brake);

    m_robotDrive = new AdvancedMecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    logger.info("The mecanum drive subsystem is initialized.");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: Implement periodic functionality such as updating dashboards/logs
    updateOdometry();
    SmartDashboard.putNumber("X Distance Traveled I Hope", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Y Distance Traveled I Hope", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Gyro Distance Traveled I Hope", odometry.getPoseMeters().getRotation().getDegrees());
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

  private Translation2d FrontLeftTranslation2d = new Translation2d(-12,10.5);
  private Translation2d FrontRightTranslation2d = new Translation2d(12,10.5);
  private Translation2d BackLeftTranslation2d = new Translation2d(-12,-10.5);
  private Translation2d BackRightTranslation2d = new Translation2d(12,-10.5);

  private  Rotation2d Gyro = new Rotation2d(0);

  private MecanumDriveKinematics kinetics = new MecanumDriveKinematics(FrontLeftTranslation2d, FrontRightTranslation2d,
       BackLeftTranslation2d, BackRightTranslation2d);
  private MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinetics, Gyro);

  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        frontLeft.getSelectedSensorVelocity()/2048*15.24,
        frontRight.getSelectedSensorVelocity()/2048*15.24,
        rearLeft.getSelectedSensorVelocity()/2048*15.24,
        rearRight.getSelectedSensorPosition()/2048*15.24);
  }

  public void updateOdometry(){
    Gyro = new Rotation2d(RobotContainer.getInstance().Gyro.getAngle());
    odometry.update(Gyro, getCurrentState());
  }
}
