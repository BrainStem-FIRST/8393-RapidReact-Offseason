package frc.robot.commands;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;


public class TransferCommand extends CommandBase{
    private TransferSubsystem transferSubsystem;
    private double speed;

    public TransferCommand(TransferSubsystem transferSubsystem, double speed){
        this.transferSubsystem = transferSubsystem;
        this.speed = speed;
        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        transferSubsystem.transfer_motor.set(speed);

    }

    @Override 
    public void end(boolean interrupted){
        transferSubsystem.transfer_motor.set(0);

    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}
