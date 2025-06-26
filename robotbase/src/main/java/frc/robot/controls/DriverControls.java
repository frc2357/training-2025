package frc.robot.controls;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerAxis;
import frc.robot.commands.drive.FlipPerspective;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {

  private CommandXboxController m_controller;

  public DriverControls() {
    m_controller = new CommandXboxController(CONTROLLER.DRIVE_CONTROLLER_PORT);
    mapControls();
  }

  public void mapControls() {
    m_controller
      .start()
      .onTrue(Robot.swerve.runOnce(() -> Robot.swerve.seedFieldCentric()));
    m_controller.back().onTrue(new FlipPerspective());

    m_controller
      .rightTrigger()
      .onTrue(new CoralRunnerAxis(this::getRightTriggerAxis));

    m_controller
      .leftTrigger()
      .onTrue(new CoralRunnerAxis(this::getLeftTriggerAxis));
  }

  public double getRightX() {
    return modifyAxis(m_controller.getRightX());
  }

  public double getLeftX() {
    return modifyAxis(m_controller.getLeftX());
  }

  public double getLeftY() {
    return modifyAxis(m_controller.getLeftY());
  }

  public Dimensionless getRightTriggerAxis() {
    return Percent.of(-m_controller.getRightTriggerAxis());
  }

  public Dimensionless getLeftTriggerAxis() {
    return Percent.of(m_controller.getLeftTriggerAxis());
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
    value = deadband(value, CONTROLLER.DRIVE_CONTROLLER_DEADBAND);
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
