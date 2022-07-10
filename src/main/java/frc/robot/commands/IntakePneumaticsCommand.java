package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePneumaticsCommand extends ParallelCommandGroup{
    private final IntakeSubsystem intakeSubsystem;
    public IntakePneumaticsCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addCommands();
        addRequirements(intakeSubsystem);
    }


    @Override
    public void execute() {
        
    }
    @Override
    public void end(boolean interrupted) {
       
    }
}
