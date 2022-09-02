package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class DefaultTransferCommand extends CommandBase {
    private TransferSubsystem transferSubsystem;
    private DoubleSupplier turnOn;
    private DoubleSupplier turnOnReversed;
    private double triggerThreshold;

    public DefaultTransferCommand(TransferSubsystem transferSubsystem, DoubleSupplier turnOn,
            DoubleSupplier turnOnReversed,
            double triggerThreshold) {
        this.transferSubsystem = transferSubsystem;
        this.turnOn = turnOn;
        this.turnOnReversed = turnOnReversed;
        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize() {
        transferSubsystem.initializeTransfer();
    }

    @Override
    public void execute() {
        boolean on = turnOn.getAsDouble() > triggerThreshold;
        boolean reversed = turnOnReversed.getAsDouble() > triggerThreshold;
        transferSubsystem.executeTransfer(on, reversed);
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.endTransfer();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}