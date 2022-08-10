// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ConstraintsConstants;
import frc.robot.Constants.Driver1ControllerConstants;
import frc.robot.Constants.Driver2ControllerConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.DefaultTransferCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class RobotContainer {

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();

  private final Joystick driver1Controller = new Joystick(Driver1ControllerConstants.CONTROLLER_PORT);
  private final Joystick driver2Controller = new Joystick(Driver2ControllerConstants.CONTROLLER_PORT);
  private final JoystickButton driver1button = new JoystickButton(driver1Controller, JoystickConstants.X_BUTTON);

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement

    // Right stick X axis -> rotation
    /*
     * drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
     * 
     * drivetrainSubsystem,
     * () -> -modifyAxis(driver1Controller.getLeftY()) *
     * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND,
     * () -> -modifyAxis(driver1Controller.getLeftX()) *
     * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND,
     * () -> -modifyAxis(driver1Controller.getRightX())
     * ConstraintsConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
     */

    /*
     * drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
     * drivetrainSubsystem,
     * () -> -driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_Y_AXIS),
     * () -> driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_X_AXIS),
     * () -> driver1Controller.getRawAxis(JoystickConstants.RIGHT_STICK_X_AXIS),
     * () -> !driver1Controller.getRawButton(JoystickConstants.LEFT_BUMPER)));
     */
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> modifyAxis(driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_Y_AXIS))
            * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> modifyAxis(driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_X_AXIS))
            * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driver1Controller.getRawAxis(JoystickConstants.RIGHT_STICK_X_AXIS))
            * ConstraintsConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(intakeSubsystem,
        () -> driver1Controller.getRawAxis(JoystickConstants.RIGHT_TRIGGER),
        Driver1ControllerConstants.TRIGGER_ACTIVATION_THRESHOLD));

    // driver1button.toggleWhenPressed(new DefaultIntakeCommand(intakeSubsystem,
    // 1.0, 0.2));
    shooterSubsystem.setDefaultCommand(
        new DefaultShooterCommand(shooterSubsystem,
            () -> driver2Controller.getRawAxis(JoystickConstants.RIGHT_TRIGGER),
            () -> driver2Controller.getRawAxis(JoystickConstants.RIGHT_STICK_X_AXIS),
            () -> driver2Controller.getRawAxis(JoystickConstants.RIGHT_STICK_Y_AXIS),
            Driver2ControllerConstants.TRIGGER_ACTIVATION_THRESHOLD,
            Driver2ControllerConstants.CONTROLLER_DEADZONE,
            false));
    transferSubsystem.setDefaultCommand(new DefaultTransferCommand(transferSubsystem,
        () -> driver1Controller.getRawAxis(JoystickConstants.RIGHT_TRIGGER),
        Driver1ControllerConstants.TRIGGER_ACTIVATION_THRESHOLD));

    // Configure the button bindings
    configureButtonBindings();
    // COMMANDS
    /*
     * new CompressorCommand(compressorSubsystem,
     * Constants.PnuematicsConstants.COMPRESSOR_MIN_PRESSURE,
     * Constants.PnuematicsConstants.COMPRESSOR_MAX_PRESSURE);
     * new IntakeCommand(intakeSubsystem, 0, false);
     */

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    /*
     * new Button(driver1Controller::getBackButton)
     * .whenPressed(drivetrainSubsystem::zeroHeading);
     * // INTAKE CONTROLS
     * new Button(driver1Controller::getYButton)
     * .whenActive(() -> intakeSubsystem.toggleIntake(true)); //FIXME//FIGURE OUT
     * HOW TO PASS IN DIFFERENT PARAMETERS WHEN ACTIVE IS FALSE
     * // TRANSFER CONTROLS
     * new Button(driver1Controller::getXButton)
     * .whenActive(() -> transferSubsystem.toggleTransfer(true)); //FIXME
     * 
     * new Button(driver1Controller::getAButton)
     * .whenActive(() -> liftCommandButton.buttonHit()); // may need to change to
     * whenpressed //FIXME
     */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    /*
     * TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
     * AutoConstants.autoMaxSpeedMetersPerSecond,
     * AutoConstants.autoMaxAccelerationMetersPerSecondSquared).setKinematics(
     * AutoConstants.autoDriveKinematics);
     * Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,
     * 0, new Rotation2d(0)),
     * List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
     * new Pose2d(2, -1,
     * Rotation2d.fromDegrees(180)),
     * trajectoryConfig);
     * 
     * PIDController xController = new PIDController(AutoConstants.autoXController,
     * 0, 0);
     * PIDController yController = new PIDController(AutoConstants.autoYController,
     * 0, 0);
     * ProfiledPIDController thetaController = new ProfiledPIDController(
     * AutoConstants.autoThetaController, 0, 0,
     * Constants.AutoConstants.autoThetaControllerConstraints);
     * thetaController.enableContinuousInput(-Math.PI, Math.PI);
     * SwerveControllerCommand swerveControllerCommand = new
     * SwerveControllerCommand(
     * trajectory,
     * drivetrainSubsystem::getPose,
     * AutoConstants.autoDriveKinematics,
     * xController,
     * yController,
     * thetaController,
     * drivetrainSubsystem::setModuleStates,
     * drivetrainSubsystem);
     * 
     * return new SequentialCommandGroup(
     * new InstantCommand(() ->
     * drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
     * swerveControllerCommand,
     * new InstantCommand(() -> drivetrainSubsystem.stopModules()));
     * 
     * // An ExampleCommand will run in autonomous
     * 
     * m_drivetrainSubsystem.setDefaultCommand(new PathCommand(
     * m_drivetrainSubsystem,
     * () -> 0,
     * () -> 0,
     * () -> 20
     * ));
     * 
     * return new InstantCommand();
     */
    // drivetrainSubsystem.setDefaultCommand(new
    // DrivetrainTestCommand(drivetrainSubsystem, 1000));

    return new InstantCommand();

  }

  private static double deadband(double value, double deadband) {
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

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
