package frc.robot.commands.coralRunner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CoralRunnerStop extends Command {

  public CoralRunenrStop() {
    addRequirements(Robot.CoralRunner);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.coralRunner.stop();
  }
}
