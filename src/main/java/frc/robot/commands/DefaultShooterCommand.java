package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;

    private DoubleSupplier shooterSpeed;
    private DoubleSupplier elevatorSpeed;
    private DoubleSupplier turretSpeed;
    private double triggerThreshold;
    private boolean isAuto;
    private double controllerDeadzone;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shootingSpeed,
            DoubleSupplier elevatorSpeed, DoubleSupplier turretSpeed, double triggerThreshold,
            double controllerDeadzone, boolean isAuto) {

        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shootingSpeed;
        this.elevatorSpeed = elevatorSpeed;
        this.turretSpeed = turretSpeed;
        this.triggerThreshold = triggerThreshold;
        this.controllerDeadzone = controllerDeadzone;
        this.isAuto = isAuto;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.initAllMotors();
    }

    @Override
    public void execute() {
        double shooterSpeedDouble = Math.abs(shooterSpeed.getAsDouble()) > triggerThreshold ? ShooterConstants.SHOOTING_MOTORS_SPEED : 0.0;
        double elevatorSpeedDouble = Math.abs(elevatorSpeed.getAsDouble()) > controllerDeadzone ? elevatorSpeed.getAsDouble() : 0.0;
        double turretSpeedDouble = Math.abs(turretSpeed.getAsDouble()) > controllerDeadzone ? turretSpeed.getAsDouble() : 0.0;
        shooterSubsystem.executeAllMotors(shooterSpeedDouble, elevatorSpeedDouble, turretSpeedDouble);
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
