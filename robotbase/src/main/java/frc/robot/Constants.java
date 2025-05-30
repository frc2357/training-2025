package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public final class Constants {

  public static final class CONTROLLER {

    public static final int DRIVE_CONTROLLER_PORT = 1;
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.01;
    public static final int CODRIVER_CONTROLLER_PORT = 0;
    public static final double CODRIVE_CONTROLLER_DEADBAND = 0.025;
    public static final double DRIVE_RUMBLE_INTENSITY = .5;
    public static final double CODRIVE_RUMBLE_INTENSITY = .5;
    public static final double DRIVE_RUMBLE_SECONDS = 2;
    public static final double CODRIVE_RUMBLE_SECONDS = 2;
  }

  public static final class SWERVE {

    public static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(1);
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;

    public static final Dimensionless AXIS_MAX_ANGULAR_RATE = Units.Percent.of(.5);
    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(.5);
  }

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

  }
}
