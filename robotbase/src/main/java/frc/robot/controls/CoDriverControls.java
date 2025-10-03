package frc.robot.controls;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Robot;
import frc.robot.commands.AlgaeKnocker.AlgaeKnockerSpeed;
import frc.robot.commands.Elevator.ElevatorAxis;
import frc.robot.commands.Elevator.ElevatorSetDistance;
import frc.robot.commands.coralRunner.CoralRunnerAxis;
import frc.robot.commands.laterator.LateratorAxis;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.commands.laterator.LateratorSetSpeed;
import frc.robot.commands.laterator.LateratorZero;
import frc.robot.controls.util.RumbleInterface;

public class CoDriverControls implements RumbleInterface {

  private CommandXboxController m_controller;

  private MutDimensionless m_rightTrigger = Percent.mutable(0);
  private MutDimensionless m_leftTrigger = Percent.mutable(0);
  private MutDimensionless m_leftY = Percent.mutable(0);

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
      .onTrue(
        new LateratorSetDistance(Constants.LATERATOR.SETPOINT.INTAKE).andThen(
          new ElevatorSetDistance(ELEVATOR.SETPOINT.INTAKE)
        )
      );
    m_controller
      .a()
      .onTrue(new LateratorSetSpeed(Constants.LATERATOR.ZERO_SPEED.times(-1)))
      .onFalse(new LateratorZero());

    m_controller
      .x()
      .and(m_controller.povRight())
      .onTrue(
        new InstantCommand(() -> {
          Robot.laterator.setZero();
          Robot.elevator.setZero();
        })
      );

    m_controller
      .x()
      .and(m_controller.povDown())
      .onTrue(
        new LateratorSetDistance(Constants.LATERATOR.SETPOINT.L3).andThen(
          new ElevatorSetDistance(ELEVATOR.SETPOINT.L3)
        )
      );
    m_controller
      .rightTrigger()
      .onTrue(new CoralRunnerAxis(this::getRightTriggerAxis));

    m_controller
      .leftTrigger()
      .onTrue(new CoralRunnerAxis(this::getLeftTriggerAxis));

    m_controller.x().whileTrue(new ElevatorAxis(this::getLeftY));

    m_controller
      .b()
      .onTrue(
        new InstantCommand(() -> {
          Robot.elevator.setZero();
        })
      );
    m_controller
      .y()
      .whileTrue(new AlgaeKnockerSpeed(Constants.ALGAE_KNOCKER.DE_ALGAE_SPEED));
  }

  public Dimensionless getRightX() {
    return Value.of(modifyAxis(m_controller.getRightX()));
  }

  public Dimensionless getLeftX() {
    return Value.of(modifyAxis(m_controller.getLeftX()));
  }

  public Dimensionless getLeftY() {
    return m_rightTrigger.mut_replace(-m_controller.getLeftY(), Value);
  }

  public Dimensionless getRightTriggerAxis() {
    return m_rightTrigger.mut_replace(
      -m_controller.getRightTriggerAxis(),
      Value
    );
  }

  public Dimensionless getLeftTriggerAxis() {
    return m_rightTrigger.mut_replace(m_controller.getLeftTriggerAxis(), Value);
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
    value = deadband(value, CONTROLLER.CODRIVER_CONTROLLER_DEADBAND);
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
