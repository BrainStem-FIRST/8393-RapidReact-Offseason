package frc.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TransferCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class ElevatorTurretParallelCommand extends ParallelCommandGroup  {

public ElevatorTurretParallelCommand (TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem, double turretSetPoint, double elevatorSetPoint, double shooterSpeed, double transferSpeed) {

addCommands(new TransferCommand(transferSubsystem, transferSpeed), new ShooterCommand(shooterSubsystem, shooterSpeed), new ElevatorCommand(shooterSubsystem, elevatorSetPoint), new TurretCommand(shooterSubsystem, turretSetPoint));










}









    
}

