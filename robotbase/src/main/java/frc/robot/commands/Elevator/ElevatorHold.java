package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorHold extends Command {

  public ElevatorHold() {
    addRequirements(Robot.elevator);
  }

  @Override
  public void initialize() {
    Robot.elevator.setSpeed(Constants.ELEVATOR.HOLD_PERCENT_OUTPUT);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.elevator.stop();
  }
}
