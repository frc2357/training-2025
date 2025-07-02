// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CORAL_RUNNER;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeKnocker;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRunner;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private static Command m_defaultDrive;

  public static CommandSwerveDrivetrain swerve;
  private static DriverControls driverControls;
  public static CoralRunner coralRunner;
  public static AlgaeKnocker algaeKnocker;

  private final Telemetry logger = new Telemetry(
    Constants.SWERVE.MAX_SPEED.in(Units.MetersPerSecond)
  );

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    swerve = TunerConstants.createDrivetrain();
    coralRunner = new CoralRunner();
    driverControls = new DriverControls();

    m_defaultDrive = new DefaultDrive(
      driverControls::getLeftX,
      driverControls::getLeftY,
      driverControls::getRightX
    );

    this.addPeriodic(
        () -> {
          coralRunner.calculateBeamBreaks();
        },
        CORAL_RUNNER.SENSOR_PERIODIC_TIME,
        CORAL_RUNNER.SENSOR_PERIODIC_OFFSET_TIME
      );

    // TODO: add alliance-dependent pose reset on roborio startup
    Robot.swerve.setDefaultCommand(m_defaultDrive);

    Robot.swerve.registerTelemetry(logger::telemeterize);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putBoolean(
      "CoralRunner isOuttakeBeamBroken",
      coralRunner.isIntakeBeamBroken()
    );
    SmartDashboard.putBoolean(
      "CoralRunner isIntakeBeamBroken",
      coralRunner.isOuttakeBeamBroken()
    );
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
