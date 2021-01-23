package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.logging.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestMotorCommand_CAN1618 extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Logger logger = Logger.getLogger(this.getClass().getName());

  public TestMotorCommand_CAN1618() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getInstance().shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.info("got to Shooter motor Activate");
    RobotContainer.getInstance().shooter.LowGoalSpin();
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").forceSetNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Retrieve the desired motor speed
    RobotContainer.getInstance().shooter.LowGoalSpin();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.info("got to Shooter Motor Spin Stop");
    RobotContainer.getInstance().shooter.Stop();
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").forceSetNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}