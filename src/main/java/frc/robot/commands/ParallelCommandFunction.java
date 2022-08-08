package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.DefaultTransferCommand;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class ParallelCommandFunction extends SequentialCommandGroup  {

public ParallelCommandFunction (TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem, boolean turnOn, double shooterSpeed, double elevatorSetPoint, double turretSetPoint) {
    new SequentialCommandGroup(new DefaultTransferCommand(transferSubsystem, turnOn), 
    
    new DefaultShooterCommand(shooterSubsystem, shooterSpeed, elevatorSetPoint, turretSetPoint));
    


}
}