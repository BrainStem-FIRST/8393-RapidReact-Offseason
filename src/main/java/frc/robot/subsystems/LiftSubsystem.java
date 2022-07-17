package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.PnuematicsConstants;

public class LiftSubsystem extends SubsystemBase{

    private final CANSparkMax innerHooksMotor = new CANSparkMax(LiftConstants.INNER_HOOKS_PORT, MotorType.kBrushless);
    private RelativeEncoder innerHooksEncoder;
    private final CANSparkMax outerHooksMotor = new CANSparkMax(LiftConstants.OUTER_HOOKS_PORT, MotorType.kBrushless);
    private RelativeEncoder outerHooksEncoder;  
    PIDController innerHooksPIDController = new PIDController(1.17, 0.0017, 0);
    PIDController outerHooksPIDController = new PIDController(1.17, 0.0017, 0);

    DoubleSolenoid pneumatics_1 = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT, PneumaticsModuleType.REVPH, 
        LiftConstants.LIFT_DS_CHANNEL_1_1, LiftConstants.LIFT_DS_CHANNEL_1_2);

    DoubleSolenoid pneumatics_2 = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT, PneumaticsModuleType.REVPH, 
        LiftConstants.LIFT_DS_CHANNEL_2_1, LiftConstants.LIFT_DS_CHANNEL_2_2);


    public LiftSubsystem(){
        ShuffleboardTab tab = Shuffleboard.getTab("Lift");
        tab.add("Inner Hooks Encoder Position", innerHooksEncoder.getPosition());
        tab.add("Inner Hooks Encoder Velocity", innerHooksEncoder.getVelocity());

        tab.add("Outer Hooks Encoder Position", outerHooksEncoder.getPosition());
        tab.add("Outer Hooks Encoder Velocity", outerHooksEncoder.getVelocity());   

        tab.add("Pneumatics 1 State", pneumatics_1.get());
        tab.add("Pneumatics 2 State", pneumatics_2.get());
    }

    //PREDICATES
    private boolean arePneumaticsExtended(){
        boolean pneumatics1check = false;
        boolean pneumatics2check = false;
        if (pneumatics_1.get() == DoubleSolenoid.Value.kForward){
            pneumatics1check = true;
        }

        if (pneumatics_2.get() == DoubleSolenoid.Value.kForward){
            pneumatics2check = true;
        }

        if (pneumatics1check && pneumatics2check){
            return true;
        } else {
            return false;
        }
    }

    private boolean arePneumaticsRetracted(){
        boolean pneumatics1check = false;
        boolean pneumatics2check = false;
        if (pneumatics_1.get() == DoubleSolenoid.Value.kReverse){
            pneumatics1check = true;
        }

        if (pneumatics_2.get() == DoubleSolenoid.Value.kReverse){
            pneumatics2check = true;
        }

        if (pneumatics1check && pneumatics2check){
            return true;
        } else {
            return false;
        }
    }



    // ENCODERS
    public void initEncoder(){
        innerHooksEncoder = innerHooksMotor.getEncoder();
        outerHooksEncoder = outerHooksMotor.getEncoder();
    }

    public void resetEncoders(){
        innerHooksEncoder.setPosition(0);
        outerHooksEncoder.setPosition(0);

    }

    public double getInnerPos(){
        return innerHooksEncoder.getPosition();
    }

    public double getOuterPos(){
        return outerHooksEncoder.getPosition();
    }

    public void innerHooksSetOuput(double output){
        innerHooksMotor.set(output);
    }

    public void outerHooksSetOutput(double output){
        outerHooksMotor.set(output);
    }

    public void moveInnerHooks(double targetPos, double tolerance){
        innerHooksPIDController.setTolerance(tolerance);
        double speed =  innerHooksPIDController.calculate(innerHooksEncoder.getPosition(), targetPos);
        innerHooksMotor.set(speed);
    }

    public void moveOuterHooks(double targetPos, double tolerance){
        outerHooksPIDController.setTolerance(tolerance);
        double speed = outerHooksPIDController.calculate(outerHooksEncoder.getPosition(), targetPos);
        outerHooksMotor.set(speed);
    }



    //PNEUMATICS
    public void pnuematicsForward(){
        pneumatics_1.set(Value.kForward);
        pneumatics_2.set(Value.kForward);
    }

    public void pnuematicsReverse(){
        pneumatics_1.set(Value.kReverse);
        pneumatics_2.set(Value.kReverse);
    }

    public void pnuematicsOff(){
        pneumatics_1.set(Value.kOff);
        pneumatics_2.set(Value.kOff);
    }

    
}
