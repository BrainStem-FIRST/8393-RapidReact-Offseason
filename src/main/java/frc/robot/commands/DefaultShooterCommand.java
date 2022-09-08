package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
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
    private double limeLightX;
    private double controllerDeadzone;
    private NetworkTableEntry limeLight;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shootingSpeed,
            DoubleSupplier shootingSpeedReversed, DoubleSupplier elevatorSpeed, DoubleSupplier turretSpeed,
            double limeLightX,
            NetworkTableEntry limeLight,
            double triggerThreshold,
            double controllerDeadzone) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shootingSpeed;
        this.shooterSpeedReversed = shootingSpeedReversed;
        this.elevatorSpeed = elevatorSpeed;
        this.turretSpeed = turretSpeed;
        this.limeLightX = limeLightX;
        this.triggerThreshold = triggerThreshold;
        this.controllerDeadzone = controllerDeadzone;
        this.limeLight = limeLight;

        addRequirements(shooterSubsystem);
    }

    // -3 to 3
    @Override
    public void initialize() {
        shooterSubsystem.initAllMotors();
    }

    @Override
    public void execute() {
        double limeLightDouble = limeLight.getDouble(0.0);
        this.limeLightX = limeLightDouble;
        double elevatorSpeedDouble = 0.0;
        if (shooterSpeed.getAsDouble() > triggerThreshold) {
            double turretSpeedDouble;
            if (this.limeLightX > 0) {
                turretSpeedDouble = 0.2;
            } else if (this.limeLightX < 0) {
                turretSpeedDouble = -0.2;
            } else {
                turretSpeedDouble = 0.0;
            }
            double shooterSpeedDouble = Math.abs(shooterSpeed.getAsDouble()) > triggerThreshold
                    ? ShooterConstants.SHOOTING_MOTORS_SPEED
                    : 0.0;
            shooterSubsystem.executeAllMotors(shooterSpeedDouble, turretSpeedDouble, elevatorSpeedDouble);
        } else if (shooterSpeedReversed.getAsDouble() > triggerThreshold) {
            double turretSpeedDouble;
            if (this.limeLightX > 0) {
                turretSpeedDouble = 0.2;
            } else if (this.limeLightX < 0) {
                turretSpeedDouble = -0.2;
            } else {
                turretSpeedDouble = 0.0;
            }
            double shooterSpeedDouble = Math.abs(shooterSpeed.getAsDouble()) > triggerThreshold
                    ? -ShooterConstants.SHOOTING_MOTORS_SPEED
                    : 0.0;
            shooterSubsystem.executeAllMotors(shooterSpeedDouble, turretSpeedDouble, elevatorSpeedDouble);
        } else {
            double turretSpeedDouble = 0.0;
            double shooterSpeedDouble = 0.0;
            shooterSubsystem.executeAllMotors(shooterSpeedDouble, turretSpeedDouble, elevatorSpeedDouble);
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
