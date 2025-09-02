package frc.robot.commands.IntakeAndScoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CORAL_RUNNER;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.Robot;
import frc.robot.commands.Elevator.ElevatorSetDistance;
import frc.robot.commands.Posing.Pose_Home;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.laterator.LateratorSetDistance;

public class Intake extends SequentialCommandGroup {

  public Intake() {
    super(
      new ParallelDeadlineGroup(
        new WaitUntilCommand(Robot.coralRunner::isIntakeBeamBroken),
        new LateratorSetDistance(LATERATOR.SETPOINT.INTAKE),
        new ElevatorSetDistance(ELEVATOR.SETPOINT.INTAKE).beforeStarting(
          new WaitUntilCommand(Robot.laterator::isAtTarget)
        ),
        new CoralRunnerSetSpeed(CORAL_RUNNER.INTAKE_SPEED)
          .beforeStarting(new WaitUntilCommand(Robot.laterator::isAtTarget))
          .beforeStarting(new WaitUntilCommand(Robot.elevator::isAtTarget))
      ),
      new ParallelCommandGroup(
        new Pose_Home(),
        new SequentialCommandGroup(
          new CoralRunnerSetSpeed(CORAL_RUNNER.INTAKE_SPEED_SLOW).until(
            Robot.coralRunner::isOuttakeBeamBroken
          ),
          new CoralRunnerSetSpeed(
            CORAL_RUNNER.INTAKE_SPEED_SLOW.times(-1)
          ).withTimeout(CORAL_RUNNER.INTAKE_SETTLE_TIME),
          new CoralRunnerSetSpeed(CORAL_RUNNER.INTAKE_SPEED_SLOW).until(
            Robot.coralRunner::isOuttakeBeamBroken
          )
        )
      )
    );
  }
}
