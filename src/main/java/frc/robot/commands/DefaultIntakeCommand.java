package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private DoubleSupplier intakeOn;
    private DoubleSupplier intakeReversed;
    private double triggerThreshold;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeOn, DoubleSupplier intakeReversed,
            double triggerThreshold) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeOn = intakeOn;
        this.intakeReversed = intakeReversed;
        this.triggerThreshold = triggerThreshold;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.initializeIntake();
    }

    @Override
    public void execute() {
        if (intakeOn.getAsDouble() > triggerThreshold) {
            intakeSubsystem.executeIntake(true, false);
        } else if (intakeReversed.getAsDouble() > triggerThreshold) {
            intakeSubsystem.executeIntake(true, true);
        }else{
            intakeSubsystem.executeIntake(false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.endIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
