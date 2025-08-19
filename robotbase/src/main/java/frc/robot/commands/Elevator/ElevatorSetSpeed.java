package frc.robot.commands.Elevator;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorSetSpeed extends Command {

  Dimensionless m_speed;

  public ElevatorSetSpeed(Dimensionless speed) {
    addRequirements(Robot.elevator);
    m_speed = speed;
  }

  @Override
  public void initialize() {
    Robot.elevator.setSpeed(m_speed);
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
