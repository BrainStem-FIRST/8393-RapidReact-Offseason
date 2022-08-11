package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class DefaultTransferCommand extends CommandBase {
    private TransferSubsystem transferSubsystem;
    private boolean turnOn;
    public DefaultTransferCommand(TransferSubsystem transferSubsystem, boolean turnOn){
        this.transferSubsystem = transferSubsystem;
        this.turnOn = turnOn;
        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize(){
        transferSubsystem.initializeTransfer();
    }

    @Override
    public void execute(){
        transferSubsystem.executeTransfer(turnOn);
    }

    @Override
    public void end(boolean interrupted){
        transferSubsystem.endTransfer();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}