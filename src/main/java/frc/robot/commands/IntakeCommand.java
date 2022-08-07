package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
/*import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase  {

    private final IntakeSubsystem intakeSubsystem;
    private double intakeAutoSpeed;
    private final boolean isAuto;
    
    public IntakeCommand(IntakeSubsystem intakeSubsystem, double setAutoIntakeSpeed, boolean isAuto){
        this.intakeSubsystem = intakeSubsystem;
        this.intakeAutoSpeed = setAutoIntakeSpeed;
        this.isAuto = isAuto;
        addRequirements(intakeSubsystem);
    }
    
    @Override 
    public void initialize(){
        intakeSubsystem.startingCommands();
    }

    @Override
    public void execute(){
        if(isAuto) intakeSubsystem.setOutput(intakeAutoSpeed);
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted){
            intakeSubsystem.endingCommands();
        }
    }

}*/
