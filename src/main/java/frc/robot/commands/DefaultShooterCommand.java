package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private DoubleSupplier shooterSpeedFunction;
    private DoubleSupplier turretSpeedFunction;
    private DoubleSupplier elevatorSpeedFunction;
    private double triggerThreshold;
    private double controllerDeadzone;
    private boolean useSetPointsInsteadOfSpeed;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterSpeedFunction,
            DoubleSupplier turretSpeedFunction,
            DoubleSupplier elevatorSpeedFunction, double triggerThreshold, double controllerDeadzone,
            boolean useSetPointsInsteadOfSpeed) {
        this.shooterSubsystem = shooterSubsystem;
        this.turretSpeedFunction = turretSpeedFunction;
        this.elevatorSpeedFunction = elevatorSpeedFunction;
        this.shooterSpeedFunction = shooterSpeedFunction;
        this.triggerThreshold = triggerThreshold;
        this.controllerDeadzone = controllerDeadzone;
        this.useSetPointsInsteadOfSpeed = useSetPointsInsteadOfSpeed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.initializeAllMotors();
    }

    @Override
    public void execute() {
        double shooterSpeed = Math.abs(shooterSpeedFunction.getAsDouble()) > triggerThreshold ? ShooterConstants.SHOOTING_MOTORS_SPEED : 0.0;
        double turretSpeed = Math.abs(turretSpeedFunction.getAsDouble()) > controllerDeadzone ? turretSpeedFunction.getAsDouble() : 0.0;
        double elevatorSpeed = Math.abs(elevatorSpeedFunction.getAsDouble()) > controllerDeadzone ? elevatorSpeedFunction.getAsDouble() : 0.0;
        shooterSubsystem.executeAllMotors(shooterSpeed, turretSpeed, elevatorSpeed);
    }

    boolean programmedWell = true;
    String isMihirHappy =  programmedWell ? "Yes" : "No";

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.endAllMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
