package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase  {
    private final IntakeSubsystem intakeSubsystem;
    public IntakeCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    
    @Override 
    public void initialize(){
        intakeSubsystem.initialize();
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted){
            intakeSubsystem.end();
        }
    }

}
