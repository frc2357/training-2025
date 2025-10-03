package frc.robot.commands.DeAlgae;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ALGAE_KNOCKER;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.Robot;
import frc.robot.commands.AlgaeKnocker.AlgaeKnockerSpeed;
import frc.robot.commands.Elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class DeAlgae_L2 extends SequentialCommandGroup {

  public DeAlgae_L2() {
    super(
      new LateratorSetDistance(LATERATOR.SETPOINT.HOME).until(
        Robot.laterator::isAtTarget
      ),
      new ElevatorSetDistance(ELEVATOR.SETPOINT.LOW_ALGAE).until(
        Robot.elevator::isAtTarget
      ),
      new ParallelCommandGroup(
        new ElevatorSetDistance(ELEVATOR.SETPOINT.LOW_ALGAE),
        new LateratorSetDistance(LATERATOR.SETPOINT.FULL_SCORING_EXTENSION),
        new AlgaeKnockerSpeed(ALGAE_KNOCKER.DE_ALGAE_SPEED)
      )
    );
  }
}
