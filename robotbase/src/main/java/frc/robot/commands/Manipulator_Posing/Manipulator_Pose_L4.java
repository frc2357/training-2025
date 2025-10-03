package frc.robot.commands.Manipulator_Posing;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.Robot;
import frc.robot.commands.Elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class Manipulator_Pose_L4 extends SequentialCommandGroup {

  public Manipulator_Pose_L4() {
    super(
      new LateratorSetDistance(LATERATOR.SETPOINT.HOME).until(
        Robot.laterator::isAtTarget
      ),
      new ParallelCommandGroup(
        new ElevatorSetDistance(ELEVATOR.SETPOINT.L4),
        new LateratorSetDistance(LATERATOR.SETPOINT.L4).beforeStarting(
          new WaitUntilCommand(Robot.elevator::isAtTarget)
        )
      )
    );
  }
}
