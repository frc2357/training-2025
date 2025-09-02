package frc.robot.controls;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ALGAE_KNOCKER;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;
import frc.robot.commands.AlgaeKnocker.AlgaeKnockerSpeed;
import frc.robot.commands.IntakeAndScoring.Intake;
import frc.robot.commands.IntakeAndScoring.ScoreAndReturn;
import frc.robot.commands.Posing.Pose_Home;
import frc.robot.commands.Posing.Pose_L2;
import frc.robot.commands.Posing.Pose_L3;
import frc.robot.commands.Posing.Pose_L4;
import frc.robot.commands.drive.FlipPerspective;
import frc.robot.commands.laterator.LateratorSetSpeed;
import frc.robot.commands.laterator.LateratorZero;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {

  private CommandXboxController m_controller;
  private MutDimensionless m_rightTrigger = Percent.mutable(0);
  private MutDimensionless m_leftTrigger = Percent.mutable(0);

  public DriverControls() {
    m_controller = new CommandXboxController(CONTROLLER.DRIVER_CONTROLLER_PORT);
    mapControls();
  }

  public void mapControls() {
    m_controller
      .start()
      .onTrue(Robot.swerve.runOnce(() -> Robot.swerve.seedFieldCentric()));
    m_controller.back().onTrue(new FlipPerspective());

    m_controller.rightTrigger().onTrue(new ScoreAndReturn());

    m_controller.leftTrigger().onTrue(new Intake());
    m_controller.leftBumper().onTrue((new Pose_Home()));
    m_controller
      .rightBumper()
      .whileTrue((new AlgaeKnockerSpeed(ALGAE_KNOCKER.DE_ALGAE_SPEED)));

    m_controller
      .a()
      .onTrue(new LateratorSetSpeed(Constants.LATERATOR.ZERO_SPEED.times(-1)))
      .onFalse(
        new LateratorZero()
          .andThen(
            new InstantCommand(() -> {
              Robot.elevator.setZero();
            })
          )
      );

    m_controller.b().onTrue(new Pose_L2());
    m_controller.x().onTrue(new Pose_L3());
    m_controller.y().onTrue(new Pose_L4());
  }

  public Dimensionless getRightX() {
    return Value.of(modifyAxis(m_controller.getRightX()));
  }

  public Dimensionless getLeftX() {
    return Value.of(modifyAxis(m_controller.getLeftX()));
  }

  public Dimensionless getLeftY() {
    return Value.of(modifyAxis(m_controller.getLeftY()));
  }

  public Dimensionless getRightTriggerAxis() {
    return m_rightTrigger.mut_replace(
      -m_controller.getRightTriggerAxis(),
      Value
    );
  }

  public Dimensionless getLeftTriggerAxis() {
    return m_leftTrigger.mut_replace(m_controller.getLeftTriggerAxis(), Value);
  }

  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private double modifyAxis(double value) {
    value = deadband(value, CONTROLLER.DRIVER_CONTROLLER_DEADBAND);
    value = Math.copySign(
      Math.pow(value, Constants.CONTROLLER.JOYSTICK_RAMP_EXPONENT),
      value
    );
    return value;
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
