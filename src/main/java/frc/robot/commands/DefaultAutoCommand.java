package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultAutoCommand extends CommandBase {

    private final DoubleSupplier translationXFunction;
    private final DoubleSupplier translationYFunction;
    private final DoubleSupplier rotationFunction;
    private DrivetrainSubsystem drivetrainSubsystem;
    private double timeInSeconds;
    private Timer timer = new Timer();

    public DefaultAutoCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier, double timeInSeconds) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXFunction = translationXSupplier;
        this.translationYFunction = translationYSupplier;
        this.rotationFunction = rotationSupplier;
        this.timeInSeconds = timeInSeconds;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXFunction.getAsDouble() * DrivetrainConstants.DRIVETRAIN_SPEED_LIMITER,
                        translationYFunction.getAsDouble() * DrivetrainConstants.DRIVETRAIN_SPEED_LIMITER,
                        rotationFunction.getAsDouble() * DrivetrainConstants.TURNING_LIMITER,
                        drivetrainSubsystem.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return timer.get() > timeInSeconds;
    }
}
