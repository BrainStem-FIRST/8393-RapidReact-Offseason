// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ConstraintsConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TransferCommand;
import frc.robot.commands.AutoCommand;

import frc.robot.commands.CompressorCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.CompressorSubsytem;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class RobotContainer {
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final CompressorSubsytem compressorSubsystem = new CompressorSubsytem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();

  private final XboxController driver1Controller = new XboxController(ControllerConstants.CONTROLLER_1_PORT);
  private final XboxController driver2Controller = new XboxController(ControllerConstants.CONTROLLER_2_PORT);

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> -modifyAxis(driver1Controller.getLeftY()) * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driver1Controller.getLeftX()) * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driver1Controller.getRightX())
            * ConstraintsConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    // default command for shooter
    //shooterSubsystem.setDefaultCommand(new TurretCommand(
      //  shooterSubsystem,
       // () -> -modifyAxis(driver2Controller.getLeftY()),
       // () -> -modifyAxis(driver2Controller.getRightY()),
       // () -> -modifyAxis(driver2Controller.getRightX())));
    // Configure the button bindings
    configureButtonBindings();
    //COMMANDS
    new CompressorCommand(compressorSubsystem, Constants.PnuematicsConstants.COMPRESSOR_MIN_PRESSURE,
        Constants.PnuematicsConstants.COMPRESSOR_MAX_PRESSURE);
    new IntakeCommand(intakeSubsystem);
    new TransferCommand(transferSubsystem);
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
    new Button(driver1Controller::getBackButton)
        .whenPressed(drivetrainSubsystem::zeroGyroscope);
    // INTAKE CONTROLS
    new Button(driver1Controller::getYButton)
        .whenActive(() -> intakeSubsystem.toggleIntake(true));
    // TRANSFER CONTROLS
    new Button(driver1Controller::getXButton)
        .whenActive(() -> transferSubsystem.toggleTransfer(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(AutoConstants.kDriveKinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
        new Pose2d(2, -1,
            Rotation2d.fromDegrees(180)),
        trajectoryConfig);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        drivetrainSubsystem::getPose,
        AutoConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        drivetrainSubsystem::setModuleStates,
        drivetrainSubsystem);

    return new SequentialCommandGroup(
        new InstantCommand(() -> drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> drivetrainSubsystem.stopModules()));

    // An ExampleCommand will run in autonomous
    /*
     * m_drivetrainSubsystem.setDefaultCommand(new PathCommand(
     * m_drivetrainSubsystem,
     * () -> 0,
     * () -> 0,
     * () -> 20
     * ));
     * 
     * return new InstantCommand();
     */

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
