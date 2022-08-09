package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase{
    private IntakeSubsystem intakeSubsystem;
    private boolean deploy;
    private double intakespeed;
    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, boolean deploy){
        this.intakeSubsystem = intakeSubsystem;
        this.deploy = deploy;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.initializeIntakeMotor();
    }

    @Override
    public void execute(){
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



