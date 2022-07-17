package frc.robot.commands;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;


public class TransferCommand extends CommandBase{
    private TransferSubsystem transferSubsystem;

    public TransferCommand(TransferSubsystem transferSubsystem){
        this.transferSubsystem = transferSubsystem;
        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize(){
        transferSubsystem.turnOff();
    }

    @Override
    public void execute(){
        transferSubsystem.turnOn();
    }

    @Override 
    public void end(boolean interrupted){
        transferSubsystem.turnOff();
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
}
