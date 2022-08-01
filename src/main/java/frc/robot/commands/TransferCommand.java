package frc.robot.commands;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;



public class TransferCommand extends CommandBase{
    private TransferSubsystem transferSubsystem;
    private double transferSpeed;

    public TransferCommand(TransferSubsystem transferSubsystem, double transferSpeed){
        this.transferSubsystem = transferSubsystem;
        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize(){
        transferSubsystem.turnOff();
    }

    @Override
    public void execute(){
        transferSubsystem.turnOnTransferColorSensor();
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
