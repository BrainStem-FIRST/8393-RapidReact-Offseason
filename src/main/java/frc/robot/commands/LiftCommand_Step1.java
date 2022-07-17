package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class LiftCommand_Step1 extends CommandBase{

    private final LiftSubsystem liftSubsystem;

    public LiftCommand_Step1(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize(){
        liftSubsystem.initEncoderandMotors();
        liftSubsystem.resetEncoders();
    }

    @Override
    public void execute(){
        liftSubsystem.moveInnerHooks(1000, 3); // FIXME - target pos
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}

// two hanging motors - same speed same power - max height
// drive backwards - not too much 
// pull down
// raise the elevator 
// extend the n[neumatics 
// fully extended (fix timing)
// once on the next rung - retract the elevator and the pistons at the same time 