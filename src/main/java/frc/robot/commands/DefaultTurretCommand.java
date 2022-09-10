package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class DefaultTurretCommand extends CommandBase {
    private TurretSubsystem turretSubsystem;
    private boolean manual;
    private boolean usePID;
    private DoubleSupplier turretSpeed;
    private NetworkTableEntry limeLight;
    private double controllerDeadzone;

    public DefaultTurretCommand(TurretSubsystem turretSubsystem, boolean manual, boolean usePID,
            DoubleSupplier turretSpeed,
            NetworkTableEntry limeLight, double controllerDeadzone) {
        this.turretSubsystem = turretSubsystem;
        this.manual = manual;
        this.turretSpeed = turretSpeed;
        this.limeLight = limeLight;
        this.controllerDeadzone = controllerDeadzone;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.initializeTurretMotor();
    }

    @Override
    public void execute() {
        double updatedSpeed;
        if (manual) {
            if (Math.abs(turretSpeed.getAsDouble()) > controllerDeadzone) {
                updatedSpeed = TurretConstants.TURRET_MOTOR_REVERSED ? -turretSpeed.getAsDouble()
                        : turretSpeed.getAsDouble();
            } else {
                updatedSpeed = 0.0;
            }
        } else {
            updatedSpeed = TurretConstants.TURRET_MOTOR_REVERSED ? -automaticTurretLock(limeLight.getDouble(0.0), 0, 0)
                    : automaticTurretLock(limeLight.getDouble(0.0), 0, 0);
        }
        turretSubsystem.executeTurretMotor(updatedSpeed, usePID);
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.endTurret();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double automaticTurretLock(double limeLightXPosition, double targetRangeMin, double targetRangeMax) {
        double turretSpeedDouble;
        if (limeLightXPosition > targetRangeMax) {
            turretSpeedDouble = 0.2;
        } else if (limeLightXPosition < targetRangeMin) {
            turretSpeedDouble = -0.2;
        } else {
            turretSpeedDouble = 0.0;
        }
        return turretSpeedDouble;
    }
}
