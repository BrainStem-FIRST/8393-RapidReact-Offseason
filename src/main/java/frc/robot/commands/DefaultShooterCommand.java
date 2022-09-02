package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;

    private DoubleSupplier shooterSpeed;
    private DoubleSupplier shooterSpeedReversed;
    private DoubleSupplier elevatorSpeed;
    private DoubleSupplier turretSpeed;
    private double triggerThreshold;
    private double controllerDeadzone;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shootingSpeed,
            DoubleSupplier shootingSpeedReversed,
            DoubleSupplier elevatorSpeed, DoubleSupplier turretSpeed, double triggerThreshold,
            double controllerDeadzone) {

        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shootingSpeed;
        this.shooterSpeedReversed = shootingSpeedReversed;
        this.elevatorSpeed = elevatorSpeed;
        this.turretSpeed = turretSpeed;
        this.triggerThreshold = triggerThreshold;
        this.controllerDeadzone = controllerDeadzone;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.initAllMotors();
        double shooterSpeedDoubleInit = 0.0;
        double elevatorSpeedDoubleInit = Math.abs(elevatorSpeed.getAsDouble()) > controllerDeadzone ? elevatorSpeed.getAsDouble() : 0.0;
        double turretSpeedDoubleInit = Math.abs(turretSpeed.getAsDouble()) > controllerDeadzone ? turretSpeed.getAsDouble() : 0.0;
        shooterSubsystem.executeAllMotors(shooterSpeedDoubleInit, elevatorSpeedDoubleInit, turretSpeedDoubleInit);
    }

    @Override
    public void execute() {
        double elevatorSpeedDouble = 0.0;
        double turretSpeedDouble = Math.abs(turretSpeed.getAsDouble()) > triggerThreshold ? turretSpeed.getAsDouble()*0.1 : 0.0;
        if(shooterSpeed.getAsDouble() > triggerThreshold){
            double shooterSpeedDouble = Math.abs(shooterSpeed.getAsDouble()) > triggerThreshold ? ShooterConstants.SHOOTING_MOTORS_SPEED : 0.0;
            shooterSubsystem.executeAllMotors(shooterSpeedDouble, elevatorSpeedDouble, turretSpeedDouble);
        } else if(shooterSpeedReversed.getAsDouble() > triggerThreshold){
            double shooterSpeedDouble = Math.abs(shooterSpeed.getAsDouble()) > triggerThreshold ? -ShooterConstants.SHOOTING_MOTORS_SPEED : 0.0;
            shooterSubsystem.executeAllMotors(shooterSpeedDouble, elevatorSpeedDouble, turretSpeedDouble);
        } else{
            double shooterSpeedDouble = 0.0;
            shooterSubsystem.executeAllMotors(shooterSpeedDouble, elevatorSpeedDouble, turretSpeedDouble);
        } 
        
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.endAllMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
