package frc.robot.commands.IntakeAndScoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Posing.Pose_Home;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;

public class ScoreAndReturn extends SequentialCommandGroup {

  public ScoreAndReturn() {
    super(
      new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SCORING_SPEED).withTimeout(
        Constants.CORAL_RUNNER.SCORING_RUN_TIME
      ),
      new Pose_Home()
    );
  }
}
