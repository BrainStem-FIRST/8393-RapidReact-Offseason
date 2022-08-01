package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.LiftSubsystem;

public class LiftCommand_Step3 extends ParallelCommandGroup{

    private final LiftSubsystem liftSubsystem;

    public LiftCommand_Step3(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize(){
        liftSubsystem.initEncoderandMotors();
    }

    @Override
    public void execute(){
        liftSubsystem.moveInnerHooks(1000, 3); //FIXME - target pos
        addCommands(
            liftSubsystem.pnuematicsReverse(), 
            liftSubsystem.moveInnerHooks(0, 3)
        );

    }



    @Override
    public void end(boolean interrupted){
        liftSubsystem.hitButton = false;

    }

    @Override
    public boolean isFinished(){
        return false;
    }


    private void addCommands(Object pnuematicsReverse, Object moveInnerHooks) {
    }

    

    
    
    
}



