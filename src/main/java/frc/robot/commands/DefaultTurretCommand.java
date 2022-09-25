package frc.robot.commands;

import java.util.ArrayList;
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
    ArrayList<Double> limeLightAvg = new ArrayList<>();

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
                updatedSpeed = turretSpeed.getAsDouble() > 0 ? TurretConstants.TURRET_MOTOR_SPEED: -TurretConstants.TURRET_MOTOR_SPEED;
                updatedSpeed = TurretConstants.TURRET_MOTOR_REVERSED ? -updatedSpeed : updatedSpeed;
            } else {
                updatedSpeed = 0.0;
            }
        } else {
            updatedSpeed = TurretConstants.TURRET_MOTOR_REVERSED
                    ? -automaticTurretLock(limeLight.getDouble(0.0), -1, 1, true)
                    : automaticTurretLock(limeLight.getDouble(0.0), -1, 1, true);
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

    private double automaticTurretLock(double limeLightXPosition, double targetRangeMin, double targetRangeMax,
            boolean useAvg) {
        double turretSpeedDouble = 0;
        double limeLightSum = 0;
        if (useAvg) {
            if (limeLightAvg.size() < 50) {
                limeLightAvg.add(limeLightXPosition);
            } else {
                limeLightAvg.clear();
            }
            limeLightSum = 0;
            for (var i = 0; i < limeLightAvg.size(); i++) {
                limeLightSum = limeLightSum + limeLightAvg.get(i);
            }
            limeLightXPosition = limeLightSum / limeLightAvg.size();
        }
        if (limeLightXPosition > targetRangeMax) {
            turretSpeedDouble = 0.09*((limeLightXPosition-targetRangeMax)/2);
        } else if (limeLightXPosition < targetRangeMin) {
            turretSpeedDouble = -0.09*((targetRangeMin-limeLightXPosition)/2);
        } else {
            turretSpeedDouble = 0.0;
        }
        return turretSpeedDouble;
    }
}
