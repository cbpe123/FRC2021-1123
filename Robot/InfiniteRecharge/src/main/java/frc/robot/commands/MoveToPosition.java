package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.logging.Logger;

public class MoveToPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Logger logger = Logger.getLogger(this.getClass().getName());
  int time = 0;

  public MoveToPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getInstance().driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // logger.info("Activating shooter motors.");
    // TODO: Retrieve the desired motor speed
    RobotContainer.getInstance().driveSubsystem.setAngleFromXY(RobotContainer.getInstance().Dashboard.getPositionX(),
     RobotContainer.getInstance().Dashboard.getPositionY());
    RobotContainer.getInstance().driveSubsystem.setMagnitude(RobotContainer.getInstance().Dashboard.getPositionThrottle());
    RobotContainer.getInstance().driveSubsystem.setDrivePositionMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Retrieve the desired motor speed
    // logger.info("In StartShooterMotorExecute");
    RobotContainer.getInstance().driveSubsystem.drivePosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getInstance().driveSubsystem.turnOffDrivePositionMode();
    RobotContainer.getInstance().driveSubsystem.driveCartesian(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.getInstance().driveSubsystem.atPoint() == true){
      return true;
    }
    return false;
  }
}