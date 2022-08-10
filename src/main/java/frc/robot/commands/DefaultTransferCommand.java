package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class DefaultTransferCommand extends CommandBase {
    private TransferSubsystem transferSubsystem;
    private DoubleSupplier turnOnFunction;
    private double triggerThreshold;
    public DefaultTransferCommand(TransferSubsystem transferSubsystem, DoubleSupplier turnOnFunction, double triggerThreshold){
        this.transferSubsystem = transferSubsystem;
        this.turnOnFunction = turnOnFunction;
        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize(){
        transferSubsystem.initializeTransfer();
    }

    @Override
    public void execute(){
        boolean turnOn = turnOnFunction.getAsDouble() > triggerThreshold;
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
