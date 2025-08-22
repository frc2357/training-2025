package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.commands.Elevator.ElevatorSetDistance;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.laterator.LateratorSetDistance;

public class ScoreAndReturn extends SequentialCommandGroup {

  public ScoreAndReturn() {
    super(
      new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SCORING_SPEED),
      new WaitCommand(Constants.CORAL_RUNNER.SCORING_RUN_TIME),
      new LateratorSetDistance(LATERATOR.SETPOINT.HOME),
      new ElevatorSetDistance(ELEVATOR.SETPOINT.HOME)
    );
  }
}
