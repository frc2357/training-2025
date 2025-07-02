package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CAN_ID {

    public static final int FRONT_LEFT_DRIVE_MOTOR = 11;
    public static final int FRONT_LEFT_STEER_MOTOR = 12;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 13;
    public static final int FRONT_RIGHT_STEER_MOTOR = 14;

    public static final int BACK_LEFT_DRIVE_MOTOR = 15;
    public static final int BACK_LEFT_STEER_MOTOR = 16;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 17;
    public static final int BACK_RIGHT_STEER_MOTOR = 18;

    public static final int FRONT_LEFT_ENCODER = 19;
    public static final int FRONT_RIGHT_ENCODER = 20;
    public static final int BACK_LEFT_ENCODER = 21;
    public static final int BACK_RIGHT_ENCODER = 22;

    public static final int CORAL_RUNNER_MOTOR = 29;
    public static final int ALGAE_KNOCKER_MOTOR = 33;
  }

  public static final class CONTROLLER {

    public static final int DRIVE_CONTROLLER_PORT = 1;
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.01;
    public static final int CODRIVER_CONTROLLER_PORT = 0;
    public static final double CODRIVE_CONTROLLER_DEADBAND = 0.025;
    public static final double DRIVE_RUMBLE_INTENSITY = .5;
    public static final double CODRIVE_RUMBLE_INTENSITY = .5;
    public static final double DRIVE_RUMBLE_SECONDS = 2;
    public static final double CODRIVE_RUMBLE_SECONDS = 2;
    public static final double JOYSTICK_RAMP_EXPONENT = 1;
  }

  public static final class SWERVE {

    public static final AngularVelocity MAX_ANGULAR_RATE =
      RotationsPerSecond.of(1);
    public static final LinearVelocity MAX_SPEED =
      TunerConstants.kSpeedAt12Volts;

    public static final Dimensionless AXIS_MAX_ANGULAR_RATE = Units.Percent.of(
      .5
    );
    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(.5);
  }

  public static class DIGITAL_INPUT {

    public static final int CORAL_RUNNER_BEAM_BREAK_OUTTAKE_ID = 8;
    public static final int CORAL_RUNNER_BEAM_BREAK_INTAKE_ID = 7;
  }

  public static class CORAL_RUNNER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);
    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(50);
    public static final Time DEBOUNCE_TIME_SECONDS = Units.Seconds.of(.03);
    public static final Time SENSOR_PERIODIC_TIME = Units.Milliseconds.of(5);
    public static final Time SENSOR_PERIODIC_OFFSET_TIME =
      Units.Milliseconds.of(3);
  }

  public static class ALGAE_KNOCKER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);
    public static final Dimensionless DEALGAE_SPEED = Units.Percent.of(50);
  }
}
