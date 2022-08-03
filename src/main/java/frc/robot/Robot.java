// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftCommandButton;
import frc.robot.commands.LiftCommand_Step1;
import frc.robot.commands.LiftCommand_Step2;
import frc.robot.commands.LiftCommand_Step3;
//import frc.robot.commands.DefaultDriveCommand;
//import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.AllianceColorPresets;
import frc.robot.subsystems.HangingSteps;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();
  private RobotContainer m_robotContainer;
  HangingSteps HangingSteps;
  private final LiftCommandButton liftCommandButton = new LiftCommandButton(HangingSteps);

  // private DrivetrainSubsystem m_drivetrain;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // m_drivetrain.setDefaultCommand(new DefaultDriveCommand(
    // m_drivetrainSubsystem,
    // () -> -modifyAxis(m_controller.getLeftY()) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(m_controller.getLeftX()) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(m_controller.getRightX()) *
    // DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
   Alliance alliance = DriverStation.getAlliance();
    if (alliance == DriverStation.Alliance.Blue){
      AllianceColorPresets color = AllianceColorPresets.BLUE;
    } else if ( alliance == DriverStation.Alliance.Red){
      AllianceColorPresets color = AllianceColorPresets.RED;
    } else if ( alliance == DriverStation.Alliance.Invalid){
      AllianceColorPresets color1 = AllianceColorPresets.NULL;
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    if (liftCommandButton.getState() == HangingSteps.STEP1){
      new LiftCommand_Step1(liftSubsystem); 
    }
    if (liftCommandButton.getState() == HangingSteps.STEP2){
      new LiftCommand_Step2(liftSubsystem); 
    }
    if (liftCommandButton.getState() == HangingSteps.STEP3){
      new LiftCommand_Step3(liftSubsystem); 
    }
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
