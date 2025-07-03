package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LATERATOR;

public class Laterator extends SubsystemBase {

  private SparkMax m_motor;

  private SparkClosedLoopController m_ClosedLoopController;
  private RelativeEncoder m_encoder;

  private DigitalInput m_hallEffectSensor;
  private Debouncer m_debouncer;
  private boolean m_isHallEffectTriggered;

  private Angle m_targetRotations = Rotations.of(Double.NaN);

  public Laterator() {
    m_motor = new SparkMax(
      Constants.CAN_ID.LATERATOR_MOTOR,
      MotorType.kBrushless
    );
    m_motor.configure(
      Constants.LATERATOR.MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_ClosedLoopController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    m_hallEffectSensor = new DigitalInput(
      Constants.DIGITAL_INPUT.LATERATOR_HALL_EFFECT_ID
    );
    m_debouncer = new Debouncer(
      Constants.LATERATOR.DEBOUNCE_TIME_SECONDS.in(Millisecond)
    );
  }

  public void setSpeed(Dimensionless speed) {
    m_targetRotations = Rotations.of(Double.NaN);
    m_motor.set(speed.in(Percent));
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    m_targetRotations = Rotations.of(Double.NaN);
    m_motor.set(axisSpeed.times(LATERATOR.AXIS_MAX_SPEED).in(Percent));
  }

  public void stop() {
    m_targetRotations = Rotations.of(Double.NaN);
    m_motor.set(0);
  }

  private void setTargetRotations(Angle targetRotations) {
    m_targetRotations = targetRotations;
    m_ClosedLoopController.setReference(
      m_targetRotations.in(Rotations),
      ControlType.kPosition
    );
  }

  private Angle getRotations() {
    return Rotations.of(m_encoder.getPosition());
  }

  private boolean isAtTargetRotations() {
    return getRotations()
      .isNear(m_targetRotations, Constants.LATERATOR.MAX_ALLOWED_ERROR);
  }

  public void SetTargetDistance(Distance targetDistance) {
    setTargetRotations(DistanceToRotations(targetDistance));
  }

  public Distance getDistance() {
    return rotationsToDistance(getRotations());
  }

  public boolean isAtTargetDistance() {
    return isAtTargetRotations();
  }

  public void calculateHallEffect() {
    m_isHallEffectTriggered = m_debouncer.calculate(m_hallEffectSensor.get());
  }

  public Boolean isAtZero() {
    return m_isHallEffectTriggered;
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  private Distance rotationsToDistance(Angle rotations) {
    return null; // TODO: math this
  }

  private Angle DistanceToRotations(Distance distance) {
    return null; // TODO: math this
  }
}
