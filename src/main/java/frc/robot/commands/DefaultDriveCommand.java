package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        /*
         * m_drivetrainSubsystem.drive(
         * ChassisSpeeds.fromFieldRelativeSpeeds(
         * translationXSupplier.getAsDouble(),
         * translationYSupplier.getAsDouble(),
         * rotationSupplier.getAsDouble(),
         * drivetrainSubsystem.getGyroscopeRotation()));
         */
        drivetrainSubsystem.setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                rotationSupplier.getAsDouble(),
                drivetrainSubsystem.getGyroscopeRotation()));

    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
