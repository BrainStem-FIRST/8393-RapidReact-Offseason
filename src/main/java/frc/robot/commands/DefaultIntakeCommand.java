package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase{

    private IntakeSubsystem intakeSubsystem;
    private DoubleSupplier intakeOn;
    private double triggerThreshold;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeOn, double triggerThreshold){
        this.intakeSubsystem = intakeSubsystem;
        this.intakeOn = intakeOn;
        this.triggerThreshold = triggerThreshold;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.initializeIntake();
    }

    @Override
    public void execute(){
        intakeSubsystem.executeIntake(intakeOn.getAsDouble() > triggerThreshold);
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.endIntake();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}



