package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.subsystems.ClimbingSubsystem;

public class DefaultClimbingCommand extends CommandBase {
    private DoubleSupplier deployClimbingPneumatics;
    private boolean lowerClimbingMotors;
    private boolean raiseClimbingMotors;
    private double triggerThreshhold;
    private ClimbingSubsystem climbingSubsystem;

    public DefaultClimbingCommand(ClimbingSubsystem climbingSubsystem, DoubleSupplier deployClimbingPneumatics,
            boolean lowerClimbingMotors, boolean raiseClimbingMotors, double triggerThreshhold) {
        this.climbingSubsystem = climbingSubsystem;
        this.deployClimbingPneumatics = deployClimbingPneumatics;
        this.lowerClimbingMotors = lowerClimbingMotors;
        this.raiseClimbingMotors = raiseClimbingMotors;
        this.triggerThreshhold = triggerThreshhold;
        addRequirements(climbingSubsystem);
    }

    @Override
    public void initialize() {
        climbingSubsystem.initializeClimbingPneumatics();
        climbingSubsystem.setClimbingMotorPowers(0);
    }

    @Override
    public void execute() {

        boolean deployPneumatics = deployClimbingPneumatics.getAsDouble() > triggerThreshhold;
        climbingSubsystem.executeClimbingPneumatics(deployPneumatics);
        if (lowerClimbingMotors) {
            double climbingSpeed = ClimbingConstants.REVERSE_CLIMBING_MOTORS ? -ClimbingConstants.CLIMBING_MOTOR_SPEEDS
                    : ClimbingConstants.CLIMBING_MOTOR_SPEEDS;
                    climbingSubsystem.setClimbingMotorPowers(climbingSpeed);
        } else if (lowerClimbingMotors) {
            double climbingSpeed = ClimbingConstants.REVERSE_CLIMBING_MOTORS ? ClimbingConstants.CLIMBING_MOTOR_SPEEDS
                    : -ClimbingConstants.CLIMBING_MOTOR_SPEEDS;
                    climbingSubsystem.setClimbingMotorPowers(climbingSpeed);
        } else {
            double climbingSpeed = 0.0;
            climbingSubsystem.setClimbingMotorPowers(climbingSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbingSubsystem.setClimbingMotorPowers(0);
        // climbingSubsystem.endClimbingPneumatics();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}