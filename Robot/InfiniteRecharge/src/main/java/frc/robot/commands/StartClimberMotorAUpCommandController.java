package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.logging.Logger;

public class StartClimberMotorAUpCommandController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Logger logger = Logger.getLogger(this.getClass().getName());

  public StartClimberMotorAUpCommandController() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getInstance().Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // logger.info("Activating shooter motors.");
    // TODO: Retrieve the desired motor speed
    RobotContainer.getInstance().Climber.SpinMotorAUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Retrieve the desired motor speed
    RobotContainer.getInstance().Climber.SpinMotorAUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getInstance().Climber.StopMotorA();    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}