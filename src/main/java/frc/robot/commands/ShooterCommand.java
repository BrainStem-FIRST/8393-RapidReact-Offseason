package frc.robot.commands;

import java.lang.reflect.Method;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private double shooterSpeed;
    private double elevatorSpeed;
    private double turretSpeed;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterSpeed, DoubleSupplier elevatorSpeed,
            DoubleSupplier turretSpeed) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shooterSpeed.getAsDouble();
        this.elevatorSpeed = elevatorSpeed.getAsDouble();
        this.turretSpeed = turretSpeed.getAsDouble();
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.resetAllMotorEncoders();
        shooterSubsystem.stopAllMotors();
    }

    @Override
    public void execute() {
        shooterSubsystem.setAllMotorSpeeds(shooterSpeed, elevatorSpeed, turretSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopAllMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}