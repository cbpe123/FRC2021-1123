package frc.robot.commands;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.logging.Logger;

public class GyroTurn extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Logger logger = Logger.getLogger(this.getClass().getName());
  double GyroAngle;
  Double SetPoint;
  
  public GyroTurn() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GyroAngle = RobotContainer.getInstance().Gyro.getAngle();
    SetPoint = RobotContainer.getInstance().Dashboard.getGoToAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GyroAngle = RobotContainer.getInstance().Gyro.getAngle();
    if(SetPoint-GyroAngle <= 180 && SetPoint-GyroAngle >= 0){
      if(Math.abs(SetPoint-GyroAngle) < 10){
        RobotContainer.getInstance().driveSubsystem.driveCartesian(0, 0, 1, 0.2);
      }
      else{
        RobotContainer.getInstance().driveSubsystem.driveCartesian(0, 0, 1, 1);
      }
    }
    else{
      if(Math.abs(SetPoint-GyroAngle) < 10){
        RobotContainer.getInstance().driveSubsystem.driveCartesian(0, 0, -1, 0.2);
      }
      else{
        RobotContainer.getInstance().driveSubsystem.driveCartesian(0, 0, -1, 1);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.getInstance().Dashboard.getGoToAngle()-GyroAngle < 0.7 &&
     RobotContainer.getInstance().Dashboard.getGoToAngle()-GyroAngle > -0.7){
    }
    return false;
  }
}