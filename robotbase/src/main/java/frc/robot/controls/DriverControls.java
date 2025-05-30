package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {

  private CommandXboxController m_controller;
  private double m_deadband;
  private static Command m_defaultDrive;

  public DriverControls(CommandXboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_defaultDrive = new DefaultDrive(m_controller::getLeftX, m_controller::getLeftY, m_controller::getRightX);

    Robot.swerve.setDefaultCommand(m_defaultDrive);

    mapControls();
  }

  public void mapControls() {
    m_controller.start().onTrue(
        Robot.swerve.runOnce(() -> Robot.swerve.seedFieldCentric()));
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

  public double getRightTriggerAxis() {
    return m_controller.getRightTriggerAxis();
  }

  public double getLeftTriggerAxis() {
    return m_controller.getLeftTriggerAxis();
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

  public double modifyAxis(double value) {
    value = deadband(value, m_deadband);
    // value = Math.copySign(Math.pow(value,
    // Constants.SWERVE.TRANSLATION_RAMP_EXPONENT), value);
    return value;
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
