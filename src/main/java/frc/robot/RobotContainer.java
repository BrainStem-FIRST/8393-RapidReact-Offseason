package frc.robot;

import java.util.Arrays;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ConstraintsConstants;
import frc.robot.Constants.Driver1ControllerConstants;
import frc.robot.Constants.Driver2ControllerConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.PnuematicsConstants;
import frc.robot.commands.DefaultClimbingCommand;
import frc.robot.commands.DefaultCompressorCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.DefaultTransferCommand;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class RobotContainer {
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final CompressorSubsystem compressorSubsystem = new CompressorSubsystem();
  private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();

  private final Joystick driver1Controller = new Joystick(Driver1ControllerConstants.CONTROLLER_PORT);
  private final Joystick driver2Controller = new Joystick(Driver2ControllerConstants.CONTROLLER_PORT);

  public RobotContainer() {

    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> modifyAxis(driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_Y_AXIS))
            * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> modifyAxis(driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_X_AXIS))
            * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driver1Controller.getRawAxis(JoystickConstants.RIGHT_STICK_X_AXIS))
            * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND));

    shooterSubsystem.setDefaultCommand(new DefaultShooterCommand(shooterSubsystem,
        () -> driver2Controller.getRawAxis(JoystickConstants.RIGHT_TRIGGER),
        () -> driver2Controller.getRawAxis(JoystickConstants.RIGHT_STICK_X_AXIS),
        () -> driver2Controller.getRawAxis(JoystickConstants.RIGHT_STICK_Y_AXIS),
        Driver2ControllerConstants.TRIGGER_ACTIVATION_THRESHOLD, Driver2ControllerConstants.CONTROLLER_DEADZONE));

    transferSubsystem.setDefaultCommand(new DefaultTransferCommand(transferSubsystem,
        () -> modifyAxis(driver2Controller.getRawAxis(JoystickConstants.RIGHT_TRIGGER)),
        Driver2ControllerConstants.TRIGGER_ACTIVATION_THRESHOLD));

    intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(intakeSubsystem,
        () -> driver1Controller.getRawAxis(JoystickConstants.RIGHT_TRIGGER),
        () -> driver1Controller.getRawAxis(JoystickConstants.LEFT_TRIGGER),
        Driver1ControllerConstants.TRIGGER_ACTIVATION_THRESHOLD));

    compressorSubsystem.setDefaultCommand(new DefaultCompressorCommand(compressorSubsystem,
        PnuematicsConstants.COMPRESSOR_MIN_PRESSURE,
        PnuematicsConstants.COMPRESSOR_MAX_PRESSURE));

    climbingSubsystem.setDefaultCommand(new DefaultClimbingCommand(climbingSubsystem,
        () -> driver2Controller.getRawAxis(JoystickConstants.LEFT_TRIGGER),
        Driver2ControllerConstants.TRIGGER_ACTIVATION_THRESHOLD));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.autoMaxSpeedMetersPerSecond,
        AutoConstants.autoMaxAccelerationMetersPerSecondSquared);
    trajectoryConfig.setKinematics(drivetrainSubsystem.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(4, 2, new Rotation2d()), new Pose2d(0, 0, new Rotation2d()),
            new Pose2d()),
        trajectoryConfig);

    PIDController xController = new PIDController(AutoConstants.autoXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.autoYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.autoThetaController, 0, 0, AutoConstants.autoThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand command = new SwerveControllerCommand(
        trajectory,
        drivetrainSubsystem::getPose,
        drivetrainSubsystem.getKinematics(),
        xController,
        yController,
        thetaController,
        drivetrainSubsystem::setModuleStates,
        drivetrainSubsystem);

    return new SequentialCommandGroup(
        new InstantCommand(() -> drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
        command,
        new InstantCommand(() -> drivetrainSubsystem.stopModules()));

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
