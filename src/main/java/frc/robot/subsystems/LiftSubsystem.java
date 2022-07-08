package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.PnuematicsConstants;

public class LiftSubsystem extends SubsystemBase{

    private final CANSparkMax innerHooksMotor = new CANSparkMax(LiftConstants.INNER_HOOKS_PORT, MotorType.kBrushless);
    private RelativeEncoder innerHooksEncoder; 
    

    private final CANSparkMax outerHooksMotor = new CANSparkMax(LiftConstants.OUTER_HOOKS_PORT, MotorType.kBrushless);
    private RelativeEncoder outerHooksEncoder; 

    DoubleSolenoid pnuematics_1 = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT, PneumaticsModuleType.REVPH, 
        PnuematicsConstants.LIFT_DS_CHANNEL_1_1, PnuematicsConstants.LIFT_DS_CHANNEL_1_2);

    DoubleSolenoid pnuematics_2 = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT, PneumaticsModuleType.REVPH, 
        PnuematicsConstants.LIFT_DS_CHANNEL_2_1, PnuematicsConstants.LIFT_DS_CHANNEL_2_2);


    public LiftSubsystem(){
        ShuffleboardTab tab = Shuffleboard.getTab("Lift");
        tab.add("Inner Hooks Encoder Position", innerHooksEncoder.getPosition());
        tab.add("Inner Hooks Encoder Velocity", innerHooksEncoder.getVelocity());

        tab.add("Outer Hooks Encoder Position", outerHooksEncoder.getPosition());
        tab.add("Outer Hooks Encoder Velocity", outerHooksEncoder.getVelocity());   
        
       

    }

    public void initEncoder(){
        innerHooksEncoder = innerHooksMotor.getEncoder();
        outerHooksEncoder = outerHooksMotor.getEncoder();
    }

    public void moveInnerHooks(double encoderTicks, double power){
        double targetpos = innerHooksEncoder.getPosition() + encoderTicks;
        while (innerHooksEncoder.getPosition() < targetpos){
            if (innerHooksEncoder.getPosition() > targetpos){
                innerHooksMotor.set(-power); 
            }
            if (innerHooksEncoder.getPosition() < targetpos){
                innerHooksMotor.set(power); 
            }
      } 
      innerHooksMotor.set(0);
    }


    public void moveOuterHooks(double encoderTicks, double power){
        double targetpos = outerHooksEncoder.getPosition() + encoderTicks;
        while (outerHooksEncoder.getPosition() < targetpos){
            if (outerHooksEncoder.getPosition() > targetpos){
                outerHooksMotor.set(-power); 
            }
            if (outerHooksEncoder.getPosition() < targetpos){
                outerHooksMotor.set(power); 
            }
      } 
      outerHooksMotor.set(0);
        }

    public void moveOuterHooksToPos(double pos, double power){
        while (outerHooksEncoder.getPosition() < pos){
            if (outerHooksEncoder.getPosition() > pos){
                outerHooksMotor.set(-power); 
            }
            if (outerHooksEncoder.getPosition() < pos){
                outerHooksMotor.set(power); 
            }
          } 
          outerHooksMotor.set(0);
    }

    public void moveInnerHooksToPos(double pos, double power){
        while (innerHooksEncoder.getPosition() < pos){
            if (innerHooksEncoder.getPosition() > pos){
                innerHooksMotor.set(-power); 
            }
            if (innerHooksEncoder.getPosition() < pos){
                innerHooksMotor.set(power); 
            }
          } 
          innerHooksMotor.set(0);
    }

    public void resetEncoders(){
        innerHooksEncoder.setPosition(0);
        outerHooksEncoder.setPosition(0);
    }


    public void pnuematics1Forward(){
        pnuematics_1.set(Value.kForward);
    }

    public void pnuematics1Reverse(){
        pnuematics_1.set(Value.kReverse);
    }

    public void pnuematics1Off(){
        pnuematics_1.set(Value.kOff);
    }


    public void pnuematics2Forward(){
        pnuematics_2.set(Value.kForward);
    }

    public void pnuematics2Reverse(){
        pnuematics_2.set(Value.kReverse);
    }

    public void pnuematics2Off(){
        pnuematics_2.set(Value.kOff);
    }

    

    
}
