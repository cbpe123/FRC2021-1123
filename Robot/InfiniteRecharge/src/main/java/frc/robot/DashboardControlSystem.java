package frc.robot;

import java.util.Map;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.LimelightSubsystem;


import frc.robot.commands.*;

public class DashboardControlSystem {

  private static NetworkTableEntry maxSpeed;
  private static NetworkTableEntry autoNum;

  private static NetworkTableEntry teleTime;
  private static NetworkTableEntry endTime;

  private static ShuffleboardTab autonomousTab;
  private static ShuffleboardTab teleopTab;
  private static ShuffleboardTab endgameTab;
  private static ShuffleboardTab Constants;

  public static void initialize() {
    Logger logger = Logger.getLogger(frc.robot.DashboardControlSystem.class.getName());

    // autonomousTab = Shuffleboard.getTab("Autonomous")
    //   .add("Select Autonomous", 0)
    //   .withWidget(BuiltInWidgets.kNumberSlider)
    //   .withProperties(Map.of("min", 0, "max", 5))
    //   .getEntry();

    NetworkTableEntry autonomous = tab.add("Select Autonomous", 0)
      .getEntry();    
    NetworkTableEntry shootingSpeed = tab.add("Shooter Motor Speed", 7200)
      .getEntry();

    // TODO: Add controls for autonomous mode
    // ShuffleboardTab Autonomous = Shuffleboard.getTab("Autonomous Tab");
    // Autonomous.add("Autonomous 1", new SetAutonomous1());
    // Autonomous.add("Autonomous 2", new SetAutonomous2());
    // Autonomous.add("Autonomous 3", new SetAutonomous3());
    // autoNum = Autonomous.add("Auto", "low")
    // .withWidget(BuiltInWidgets.kNumberBar)
    // .getEntry();

    teleopTab = Shuffleboard.getTab("Teleop");
    // teleopTab.add("Limelight", new LimelightCommand(new LimelightSubsystem()));
    // teleopTab.add("Intake Extend", new ExtendIntakePiston());
    // teleopTab.add("Intake Retract", new RetractIntakePiston());
    // teleopTab.add("Calibrate Gyro", new CalibrateGyro());
    ShuffleboardLayout motorControl = teleopTab.getLayout("Motor Control", BuiltInLayouts.kList)
      .withPosition(0, 0).withSize(2, 2)
      .withProperties(Map.of("Label Position", "HIDDEN"));

    motorControl.add("Spin Motors", new SpinShooterMotorsCommand());
    motorControl.add("Shooter Motors Start", new StartShooterMotorsCommand());
    motorControl.add("Shooter Motors Stop", new StopShooterMotorsCommand());
    
    ShuffleboardLayout motorSpeed = teleopTab.getLayout("Motor Speed", BuiltInLayouts.kList)
      .withPosition(2, 0).withSize(2, 3)
      .withProperties(Map.of("Label Position", "HIDDEN"));

    // motorSpeed.add("Increase Shooter Motor Speed 50", new IncreaseShooterMotorSpeed50());
    motorSpeed.add("Increase Shooter Motor Speed 100", new IncreaseShooterMotorSpeed100());
    motorSpeed.add("Increase Shooter Motor Speed 500", new IncreaseShooterMotorSpeed500());
    motorSpeed.add("Increase Shooter Motor Speed 1000", new IncreaseShooterMotorSpeed1000());
    // motorSpeed.add("Decrease Shooter Motor Speed 50", new DecreaseShooterMotorSpeed50());
    motorSpeed.add("Decrease Shooter Motor Speed 100", new DecreaseShooterMotorSpeed100());
    motorSpeed.add("Decrease Shooter Motor Speed 500", new DecreaseShooterMotorSpeed500());
    motorSpeed.add("Decrease Shooter Motor Speed 1000", new DecreaseShooterMotorSpeed1000());
    motorSpeed.add("High Goal Speed Set", new SetShooterMotorSpeedHighGoal());

    // maxSpeed = motorSpeed.add("Speed Slider", 1)
    //   .withWidget(BuiltInWidgets.kNumberSlider)
    //   .withProperties(Map.of("min", 0, "max", 1))
    //   .getEntry();

    ShuffleboardLayout ramControl = teleopTab.getLayout("Shooter Stuffz", BuiltInLayouts.kList)
      .withPosition(6, 3).withSize(2, 1)
      .withProperties(Map.of("Label Position", "HIDDEN"));

    teleopTab.add("Shoot One Ball", new ShootOneBallCommand())
      .withPosition(4, 1).withSize(2, 1)
      .withProperties(Map.of("Label Position", "HIDDEN"));

    ramControl.add("Load", new ShooterLoadCommand());
    ramControl.add("Shoot Lower Goal 6", new LowerGoalShootCommand());
    // .withPosition(4, 3).withSize(2, 1)
    //   .withProperties(Map.of("Label Position", "HIDDEN"));
      ramControl.add("Shoot Lower Goal 10 Balls", new LowerGoalShootALotCommand());
    // .withPosition(4, 2).withSize(2, 1)
    //   .withProperties(Map.of("Label Position", "HIDDEN"));
    teleopTab.add("Shoot High Goal All", new ShootCommand())
    .withPosition(4, 0).withSize(2, 1)
      .withProperties(Map.of("Label Position", "HIDDEN"));
    ramControl.add("Line Up on goal", new LineUpOnGoal());
    ramControl.add("Rotate to Goal", new RotateToGoal());
    // .withPosition(6, 4).withSize(2, 1)
    //   .withProperties(Map.of("Label Position", "HIDDEN"));
    ramControl.add("Shoot High And Rotate to goal", new ShootHighAndAimOnGoal());

    ShuffleboardLayout Misc = teleopTab.getLayout("Misc", BuiltInLayouts.kList)
      .withPosition(6, 0).withSize(2, 2)
      .withProperties(Map.of("Label Position", "HIDDEN"));

    Misc.add("Limelight", new LimelightCommand(new LimelightSubsystem()));
    Misc.add("Intake Extend", new ExtendIntakePiston());
    Misc.add("Intake Retract", new RetractIntakePiston());
    Misc.add("Calibrate Gyro", new CalibrateGyro());

    teleTime = teleopTab.add("Time Left", 135)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", 0, "max", 135))
      .withPosition(0, 2).withSize(2, 1)
      .getEntry();

    endgameTab = Shuffleboard.getTab("Endgame");
    // endgameTab.add("Deploy hook", false);
    // endgameTab.add("Winch left", false);
    // endgameTab.add("Winch right", false);
    // endgameTab.add("Start Climber Motors Together Up" , new StartClimberMotorsTogetherUpCommand());
    // endgameTab.add("Stop Climber Motors Together" , new StopClimberMotorsTogetherCommand());
    // endgameTab.add("Start Climber Motors Together Down" , new StartClimberMotorsTogetherDownCommand());
    // endgameTab.add("Start Climber Motors Together Down Slow" , new StartClimberMotorsTogetherDownSlowCommand());
    // endgameTab.add("Start Left Climber Up" , new StartClimberMotorAUpCommand());
    // endgameTab.add("Stop Left Climber" , new StopClimberMotorACommand());
    // endgameTab.add("Start Left Climber Down" , new StartClimberMotorADownCommand());
    // endgameTab.add("Start Right Climber Up" , new StartClimberMotorBUpCommand());
    // endgameTab.add("Stop Right Climber" , new StopClimberMotorBCommand());
    // endgameTab.add("Start Right Climber Down" , new StartClimberMotorBDownCommand());
    // endgameTab.add("Release the Winch", new ReleaseWinchCommand());
    // endgameTab.add("Engage the Winch", new EngageWinchCommand());
    // endgameTab.add("Go Right Slowly", new ClimbingMovingRight());
    // endgameTab.add("Go Left Slowly", new ClimbingMovingLeft());
    // endgameTab.add("Stop Drivetrain", new ClimbingMovingStop());
    // ShuffleboardLayout BothMotors = endgameTab.getLayout("BothMotors", BuiltInLayouts.kList)
    //   .withPosition(0, 0).withSize(3, 2)
    //   .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Up Together" , new StartClimberMotorsTogetherUpCommand())
    .withPosition(1, 0).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Stop Together" , new StopClimberMotorsTogetherCommand())
    .withPosition(1, 1).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Down Together" , new StartClimberMotorsTogetherDownCommand())
    .withPosition(1, 2).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));

    // ShuffleboardLayout LeftMotor = endgameTab.getLayout("LeftMotor", BuiltInLayouts.kList)
    //   .withPosition(3, 0).withSize(2, 2)
    //   .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Left Up" , new StartClimberMotorAUpCommand())
    .withPosition(3, 1).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Stop Left Climber" , new StopClimberMotorACommand())
    .withPosition(3, 2).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Left Down" , new StartClimberMotorADownCommand())
    .withPosition(3, 3).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Left Down Slow" , new StartClimberMotorADownCommandSlow())
    .withPosition(3, 4).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));

    // ShuffleboardLayout RightMotor = endgameTab.getLayout("RightMotor", BuiltInLayouts.kList)
    //   .withPosition(5, 0).withSize(2, 2)
    //   .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Right Up" , new StartClimberMotorBUpCommand())
    .withPosition(5, 0).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Stop Right Climber" , new StopClimberMotorBCommand())
    .withPosition(5, 1).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Right Down" , new StartClimberMotorBDownCommand())
    .withPosition(5, 2).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Right Down Slow" , new StartClimberMotorBDownCommandSlow())
    .withPosition(5, 3).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));

    // ShuffleboardLayout Winches = endgameTab.getLayout("Winches", BuiltInLayouts.kList)
    endgameTab.add("Disable Ratchet", new ReleaseWinchCommand())
    .withPosition(7, 0).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    endgameTab.add("Enable Ratchet", new EngageWinchCommand())
    .withPosition(7, 1).withSize(2, 1)
    .withProperties(Map.of("Label Position", "HIDDEN"));

    ShuffleboardLayout Driving = endgameTab.getLayout("Driving", BuiltInLayouts.kList)
    .withPosition(7, 2).withSize(2, 2)
    .withProperties(Map.of("Label Position", "HIDDEN"));
    Driving.add("Go Right Slowly", new ClimbingMovingRight());
      // .withPosition(7, 0).withSize(2, 1)
      // .withProperties(Map.of("Label Position", "HIDDEN"));
    Driving.add("Go Left Slowly", new ClimbingMovingLeft());
    Driving.add("Stop Drivetrain", new ClimbingMovingStop());

    endTime = endgameTab.add("Time Left", 135)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", 0, "max", 135))
      .withPosition(0, 2).withSize(2, 1)
      .getEntry();

    // TODO: Add controls for end game climb
    // ShuffleboardTab EndGame = Shuffleboard.getTab("Endgame");
    // EndGame.add("Start Climber Motors Together" , new StartClimberMotorsTogetherCommand());
    // EndGame.add("Stop Climber Motors Together" , new StopClimberMotorsTogetherCommand());
    // EndGame.add("Start Left Climber" , new StartClimberMotorACommand());
    // EndGame.add("Stop Left Climber" , new StopClimberMotorACommand());
    // EndGame.add("Start Right Climber" , new StartClimberMotorBCommand());
    // EndGame.add("Stop Right Climber" , new StopClimberMotorBCommand());

    ShuffleboardTab camTab = Shuffleboard.getTab("piCamera");
    NetworkTableEntry piTable = camTab.add("Pi Camera", true)
      .withWidget(BuiltInWidgets.kCameraStream)
      //.withProperties(Map.of("min", 0, "max", 135))
      .withPosition(0, 0).withSize(2, 2)
      .getEntry();
  }
  public static double getSliderSpeed() {
    double shooterSpeed = maxSpeed.getDouble(0.0);
    SmartDashboard.putNumber("Logging Shooter Speed: ", shooterSpeed);
    return shooterSpeed;
  }

  // public static String getAutoStr() {
  //   String autoStr = autoNum.getString("high");
  //   SmartDashboard.putString("AutoNum", autoStr);
  //   return autoStr;
  // }

  public static void PutIsBox(boolean boo){
    SmartDashboard.putBoolean("Limelight Box", boo);
  }

  public static void putTimeRemaining(double time){
    teleTime.forceSetDouble(time);
    endTime.forceSetDouble(time);
  }
  public double getAutoNumber(){
    return autonomous.getDouble(-1.0)
  }
  public double getShooterSpeed(){
    return shootingSpeed.getDouble(7200)
  }
}