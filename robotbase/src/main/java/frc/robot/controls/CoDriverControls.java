package frc.robot.controls;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerAxis;
import frc.robot.commands.laterator.LateratorAxis;
import frc.robot.commands.laterator.LateratorSetpoint;
import frc.robot.controls.util.RumbleInterface;

public class CoDriverControls implements RumbleInterface {

  private CommandXboxController m_controller;

  public CoDriverControls() {
    m_controller = new CommandXboxController(
      CONTROLLER.CODRIVER_CONTROLLER_PORT
    );
    mapControls();
  }

  public void mapControls() {
    m_controller
      .x()
      .and(() -> getLeftX().gte(Percent.one()))
      .onTrue(new LateratorAxis(this::getLeftX));
    m_controller
      .x()
      .and(m_controller.povUp())
      .onTrue(new LateratorSetpoint(Constants.LATERATOR.SETPOINT.L3));

    m_controller
      .x()
      .and(m_controller.povRight())
      .onTrue(new InstantCommand(() -> Robot.laterator.setZero()));

    m_controller
      .x()
      .and(m_controller.povDown())
      .onTrue(new LateratorSetpoint(Constants.LATERATOR.SETPOINT.L2));
    m_controller
      .rightTrigger()
      .onTrue(new CoralRunnerAxis(this::getRightTriggerAxis));

    m_controller
      .leftTrigger()
      .whileTrue(new CoralRunnerAxis(this::getLeftTriggerAxis));
  }

  public Dimensionless getRightX() {
    return Percent.of(modifyAxis(m_controller.getRightX()));
  }

  public Dimensionless getLeftX() {
    return Percent.of(modifyAxis(m_controller.getLeftX()));
  }

  public Dimensionless getLeftY() {
    return Percent.of(modifyAxis(m_controller.getLeftY()));
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
    value = deadband(value, CONTROLLER.CODRIVE_CONTROLLER_DEADBAND);
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
