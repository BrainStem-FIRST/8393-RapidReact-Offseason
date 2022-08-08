package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.DefaultTransferCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class ParallelCommandFunction extends ParallelCommandGroup  {

public ParallelCommandFunction (TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem, boolean turnOn, double shooterSpeed, double elevatorSetPoint, double turretSetPoint) {

addCommands(new DefaultTransferCommand(transferSubsystem, turnOn), new ElevatorCommand(shooterSubsystem, elevatorSetPoint), new TurretCommand(shooterSubsystem, turretSetPoint), new DefaultShooterCommand(shooterSubsystem, shooterSpeed));


}
}