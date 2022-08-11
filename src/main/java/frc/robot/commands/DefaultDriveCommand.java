package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final DoubleSupplier translationXFunction;
    private final DoubleSupplier translationYFunction;
    private final DoubleSupplier rotationFunction;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXFunction = translationXSupplier;
        this.translationYFunction = translationYSupplier;
        this.rotationFunction = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }
    

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXFunction.getAsDouble()*DrivetrainConstants.DRIVETRAIN_SPEED_LIMITER,
                        translationYFunction.getAsDouble()*DrivetrainConstants.DRIVETRAIN_SPEED_LIMITER,
                        rotationFunction.getAsDouble()*DrivetrainConstants.TURNING_LIMITER,
                        drivetrainSubsystem.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
