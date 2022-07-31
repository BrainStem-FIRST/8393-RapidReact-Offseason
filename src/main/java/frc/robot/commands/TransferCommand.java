package frc.robot.commands;

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
        transferSubsystem.turnOnTransfer();
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
