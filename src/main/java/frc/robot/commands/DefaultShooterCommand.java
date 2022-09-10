package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private DoubleSupplier shooterSpeed;
    private boolean usePID;
    private DoubleSupplier shooterSpeedReversed;
    private double triggerThreshold;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterSpeed, 
            DoubleSupplier shooterSpeedReversed,
            boolean usePID,
            double triggerThreshold) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shooterSpeed;
        this.shooterSpeedReversed = shooterSpeedReversed;
        this.triggerThreshold = triggerThreshold;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.initializeShooterMotors();
    }

    @Override
    public void execute() {
        double updatedSpeed;
        if (Math.abs(shooterSpeed.getAsDouble()) > triggerThreshold) {
            updatedSpeed = ShooterConstants.SHOOTING_MOTORS_REVERSED ? -ShooterConstants.SHOOTING_MOTORS_SPEED
                    : ShooterConstants.SHOOTING_MOTORS_SPEED;
        } else if (Math.abs(shooterSpeedReversed.getAsDouble()) > triggerThreshold) {
            updatedSpeed = ShooterConstants.SHOOTING_MOTORS_REVERSED ? ShooterConstants.SHOOTING_MOTORS_SPEED
                    : -ShooterConstants.SHOOTING_MOTORS_SPEED;
        } else {
            updatedSpeed = 0.0;
        }
        shooterSubsystem.executeShooterMotors(updatedSpeed, usePID);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.endShooterMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
