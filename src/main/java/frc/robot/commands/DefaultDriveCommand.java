package frc.robot.commands;

/*import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Driver1ControllerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction;
    private final BooleanSupplier fieldOrientedFunction;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier xSpdFuncton, DoubleSupplier ySpdFunction, DoubleSupplier turningSpdFunction,
            BooleanSupplier fieldOrientedFunction) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xSpdFunction = xSpdFuncton;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        addRequirements(drivetrainSubsystem);
    }
    /*
     * private final DoubleSupplier xSpdFunction;
     * private final DoubleSupplier ySpdFunction;
     * private final DoubleSupplier turningSpdFunction;
     */

    /*
     * Public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
     * DoubleSupplier translationXSupplier,
     * DoubleSupplier translationYSupplier,
     * DoubleSupplier rotationSupplier) {
     * this.drivetrainSubsystem = drivetrainSubsystem;
     * this.translationXSupplier = translationXSupplier;
     * this.translationYSupplier = translationYSupplier;
     * this.rotationSupplier = rotationSupplier;
     * 
     * addRequirements(drivetrainSubsystem);
     * }
     

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // get realtime joystick inputs
        double xSpeed = xSpdFunction.getAsDouble();
        double ySpeed = ySpdFunction.getAsDouble();
        double turningSpeed = turningSpdFunction.getAsDouble();
        // apply controller deadzone
        xSpeed = Math.abs(xSpeed) > Driver1ControllerConstants.CONTROLLER_DEADZONE ? xSpeed : 0.0;
        ySpeed = Math.abs(xSpeed) > Driver1ControllerConstants.CONTROLLER_DEADZONE ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Driver1ControllerConstants.CONTROLLER_DEADZONE ? turningSpeed : 0.0;

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.getAsBoolean()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
                    ySpeed, turningSpeed, drivetrainSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

    

        drivetrainSubsystem.setModuleStates(chassisSpeeds);

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

        /*
         * drivetrainSubsystem.setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
         * translationXSupplier.getAsDouble(),
         * translationYSupplier.getAsDouble(),
         * rotationSupplier.getAsDouble(),
         * drivetrainSubsystem.getGyroscopeRotation()));
         
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
*/