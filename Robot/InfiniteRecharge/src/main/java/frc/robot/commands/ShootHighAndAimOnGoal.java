package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.logging.Logger;

import frc.robot.RobotContainer;
import frc.robot.commands.RotateToGoal;


/**
 * An example command that uses an example subsystem.
 */
public class ShootHighAndAimOnGoal extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Logger logger = Logger.getLogger(this.getClass().getName());
  int time = 0;
  int TimeSinceLastShot = 0;
  RotateToGoal Aim = new RotateToGoal();
  int TimeSinceBallFire = 0;
  int NumberOfBallsFired = 0;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootHighAndAimOnGoal() {
    addRequirements(RobotContainer.getInstance().shooter);
    addRequirements(RobotContainer.getInstance().intakeSubsystem);
    addRequirements(RobotContainer.getInstance().Gyro);
    // Use addRequirements() here to declare subsystem dependencies.
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.info("got to motor Activate");
    RobotContainer.getInstance().shooter.setVelocity(RobotContainer.getInstance().Dashboard.getShooterSetSpeed());
    RobotContainer.getInstance().intakeSubsystem.IntakeSlowHigh();
    RobotContainer.getInstance().shooter.ResetNumberOfBallsFired();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").forceSetNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time++;
    RobotContainer.getInstance().shooter.setVelocity(RobotContainer.getInstance().Dashboard.getShooterSetSpeed());
    if(RobotContainer.getInstance().Limelight.ifBox() == true){
      if(Aim.isFinished() == false){
        Aim.execute();
      }
      else{
       if(time>50){
          fireBalls();
        }  
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.info("got to High Goal Shoot Stop");
    RobotContainer.getInstance().shooter.Stop();
    RobotContainer.getInstance().shooter.LoadBall();
    RobotContainer.getInstance().intakeSubsystem.Stop();
    RobotContainer.getInstance().shooter.ResetNumberOfBallsFired();
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").forceSetNumber(1);
    time = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.getInstance().shooter.getNumberOfBallsFired() >= 6){
      return true;
    }
    return false;
  }
  public void fireBalls(){
    if (time - TimeSinceBallFire > 30){
      RobotContainer.getInstance().shooter.LoadBall();
      NumberOfBallsFired++;
    }

    if(time - TimeSinceBallFire > 70){
      RobotContainer.getInstance().shooter.fireBall();
      TimeSinceBallFire = time;
    }
  }
}