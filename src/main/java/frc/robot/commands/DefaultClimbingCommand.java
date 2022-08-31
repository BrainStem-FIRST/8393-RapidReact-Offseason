package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

public class DefaultClimbingCommand extends CommandBase {
    private DoubleSupplier deploy;
    private double triggerThreshhold;
    private ClimbingSubsystem climbingSubsystem;
    public DefaultClimbingCommand(ClimbingSubsystem climbingSubsystem, DoubleSupplier deploy, double triggerThreshhold){
        this.climbingSubsystem = climbingSubsystem;
        this.deploy = deploy;
        this.triggerThreshhold = triggerThreshhold;
        addRequirements(climbingSubsystem);
    }
    
    @Override
    public void initialize(){
        climbingSubsystem.initializeClimbingPneumatics();
    }

    @Override
    public void execute(){
        boolean deployPneumatics = deploy.getAsDouble() > triggerThreshhold;
        climbingSubsystem.executeClimbingPneumatics(deployPneumatics);
    }

    @Override
    public void end(boolean interrupted){
        climbingSubsystem.endClimbingPneumatics();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
