package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
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

        public static final double AXIS_MAX_ANGULAR_RATE = (.5);
        public static final double AXIS_MAX_SPEED = (.5);

    }
}
