package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase{
    private IntakeSubsystem intakeSubsystem;
    private DoubleSupplier deployFunction;
    private double triggerThreshold;
    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier deployFunction, double triggerThreshold){
        this.intakeSubsystem = intakeSubsystem;
        this.deployFunction = deployFunction;
        this.triggerThreshold = triggerThreshold;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.initializeIntakeMotor();
    }

    @Override
    public void execute(){
        boolean deploy = deployFunction.getAsDouble() > triggerThreshold;
        intakeSubsystem.executeIntake(deploy);
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

