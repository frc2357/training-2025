package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeKnocker extends SubsystemBase {

  private SparkMax m_motor;

  public AlgaeKnocker() {
    m_motor = new SparkMax(
      Constants.CAN_ID.ALGAE_KNOCKER_MOTOR,
      MotorType.kBrushless
    );
    m_motor.configure(
      Constants.ALGAE_KNOCKER.MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public void setSpeed(Dimensionless speed) {
    m_motor.set(speed.in(Value));
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
