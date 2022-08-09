package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase{
    private IntakeSubsystem intakeSubsystem;
    private boolean deploy;
    private double intakeOn;
    private double threshold;
    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, double intakeOn, double threshold){
        this.intakeSubsystem = intakeSubsystem;
        this.intakeOn = intakeOn;
        this.threshold = threshold;
        this.deploy = deploy;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.initializeIntakeMotor();
    }

    @Override
    public void execute(){
        boolean deploy = intakeOn > threshold;
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



