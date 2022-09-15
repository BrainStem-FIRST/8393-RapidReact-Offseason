package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private DoubleSupplier shooterSpeed;
    private DoubleSupplier secondaryShooterSpeed;
    private boolean usePID;
    private DoubleSupplier shooterSpeedReversed;
    private double triggerThreshold;
    private double timeInSeconds;
    private Timer timer = new Timer();

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterSpeed,
            DoubleSupplier secondaryShooterSpeed,
            DoubleSupplier shooterSpeedReversed,
            boolean usePID,
            double triggerThreshold, double timeInSeconds) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shooterSpeed;
        this.secondaryShooterSpeed = secondaryShooterSpeed;
        this.shooterSpeedReversed = shooterSpeedReversed;
        this.triggerThreshold = triggerThreshold;
        this.timeInSeconds = timeInSeconds;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.initializeShooterMotors();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double updatedSpeed;
        if ((Math.abs((shooterSpeed.getAsDouble())) > triggerThreshold)
                || (Math.abs(secondaryShooterSpeed.getAsDouble()) > triggerThreshold)) {
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
        if (timeInSeconds == 0) {
            return false;
        } else {
            return timer.get() > timeInSeconds ? true : false;
        }
    }
}
