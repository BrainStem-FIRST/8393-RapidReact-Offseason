package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class LiftCommand_Step2 extends CommandBase{

    private final LiftSubsystem liftSubsystem;

    public LiftCommand_Step2(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize(){
        liftSubsystem.initEncoderandMotors();
      
    }

    @Override
    public void execute(){
        ((Command) liftSubsystem.moveInnerHooks(0, 3)).andThen(() -> liftSubsystem.moveInnerHooks(600, 3)); //FIXME - target pos
        liftSubsystem.pnuematicsForward();
    }


    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}


// pull down
// raise the elevator 
// extend the n[neumatics 
// fully extended (fix timing)
// once on the next rung - retract the elevator and the pistons at the same time 