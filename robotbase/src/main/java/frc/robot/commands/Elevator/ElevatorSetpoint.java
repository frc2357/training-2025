package frc.robot.commands.Elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class ElevatorSetpoint extends Command {

  private Supplier<Distance> m_setpoint;

  public ElevatorSetpoint(Distance setpoint) {
    addRequirements(Robot.elevator);
    m_setpoint = () -> setpoint;
  }

  public ElevatorSetpoint(Supplier<Distance> setpoint) {
    addRequirements(Robot.elevator);
    m_setpoint = setpoint;
  }

  @Override
  public void execute() {
    Robot.elevator.setTargetDistance(m_setpoint.get());
    Robot.elevator.updateMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.elevator.setTargetDistance(Inches.of(0));
  }
}
