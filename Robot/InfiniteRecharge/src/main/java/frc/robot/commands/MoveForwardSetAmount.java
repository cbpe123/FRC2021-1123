package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.logging.Logger;

public class MoveForwardSetAmount extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Logger logger = Logger.getLogger(this.getClass().getName());
  int time = 0;

  public MoveForwardSetAmount() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // logger.info("Activating shooter motors.");
    // TODO: Retrieve the desired motor speed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Retrieve the desired motor speed
    // logger.info("In StartShooterMotorExecute");
    RobotContainer.getInstance().driveSubsystem.driveRotation(RobotContainer.getInstance().Dashboard.getDriveSetPoint());
    time++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time>3){
      return true;
    }
    return false;
  }
}