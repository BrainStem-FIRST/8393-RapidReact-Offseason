package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class CollectorTransferParallel extends ParallelCommandGroup {


public CollectorTransferParallel (IntakeSubsystem intakeSubsystem, double intakeSpeed, double threshold, TransferSubsystem transferSubsystem, boolean turnOn) {

    addCommands(new DefaultIntakeCommand(intakeSubsystem, intakeSpeed, threshold), new DefaultTransferCommand(transferSubsystem, turnOn));



}

    
}
