// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Driver1ControllerConstants;
import frc.robot.Constants.Driver2ControllerConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.CollectorTransferParallel;
import frc.robot.commands.CompressorCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.DefaultTransferCommand;
import frc.robot.subsystems.CompressorSubsytem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.Constants.JoystickConstants;

public class RobotContainer {

  //private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  

  private final Joystick driver1Controller = new Joystick(Driver1ControllerConstants.CONTROLLER_PORT);
  private final Joystick driver2Controller = new Joystick(Driver2ControllerConstants.CONTROLLER_PORT);
  private final JoystickButton driver2button = new JoystickButton(driver2Controller, JoystickConstants.X_BUTTON);

  public RobotContainer() {

    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> -driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_Y_AXIS),
        () -> driver1Controller.getRawAxis(JoystickConstants.LEFT_STICK_X_AXIS),
        () -> driver1Controller.getRawAxis(JoystickConstants.RIGHT_STICK_X_AXIS),
        () -> !driver1Controller.getRawButton(JoystickConstants.LEFT_BUMPER)));
     
        driver2button.toggleWhenPressed(new CollectorTransferParallel(intakeSubsystem, 1, 0.2, transferSubsystem, true));
        

    compressorSubsystem.setDefaultCommand(new CompressorCommand(compressorSubsystem, 100, 160));
    if(((shooterSubsystem.turretMotorEncoder.getPosition() * 42 == 0) || shooterSubsystem.turretMotorEncoder.getPosition() * 42 < 50 || shooterSubsystem.turretMotorEncoder.getPosition() * 42 > -50) 
    && ((shooterSubsystem.elevatorMotorEncoder.getPosition() == 0) || shooterSubsystem.elevatorMotorEncoder.getPosition() *42 < 50 || shooterSubsystem.elevatorMotorEncoder.getPosition() *42 > -50))
    shooterSubsystem.setDefaultCommand(new DefaultShooterCommand(shooterSubsystem, () -> driver2Controller.getRawAxis(3), () -> driver2Controller.getRawAxis(5), () -> driver2Controller.getRawAxis(1)));
    

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
