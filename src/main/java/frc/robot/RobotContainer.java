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
import frc.robot.Constants.Driver1ControllerConstants;
import frc.robot.Constants.Driver2ControllerConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.DefaultTransferCommand;
import frc.robot.commands.DrivetrainTestCommand;
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
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> -driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_Y_AXIS),
        () -> driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_X_AXIS),
        () -> driver1Controller.getRawAxis(JoystickConstants.RIGHT_STICK_X_AXIS),
        () -> !driver1Controller.getRawButton(JoystickConstants.LEFT_BUMPER)));

 
    shooterSubsystem.setDefaultCommand(
        new DefaultShooterCommand(shooterSubsystem, driver2Controller.getRawAxis(JoystickConstants.LEFT_STICK_Y_AXIS),
            driver2Controller.getRawAxis(JoystickConstants.RIGHT_STICK_X_AXIS),
            driver2Controller.getRawAxis(JoystickConstants.RIGHT_STICK_Y_AXIS)));
    transferSubsystem.setDefaultCommand(new DefaultTransferCommand(transferSubsystem, true));
  

    configureButtonBindings();


  }

 
  private void configureButtonBindings() {
      new JoystickButton(driver1Controller, 3)
        .toggleWhenActive(new DefaultShooterCommand(shooterSubsystem, 0.5, driver1Controller.getRawAxis(1), driver1Controller.getRawAxis(3)));

     
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {



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
