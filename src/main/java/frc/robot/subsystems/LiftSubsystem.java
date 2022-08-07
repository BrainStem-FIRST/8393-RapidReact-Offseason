package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.PnuematicsConstants;

public class LiftSubsystem extends SubsystemBase implements AutoCloseable {

    private final CANSparkMax innerHooksMotor1 = new CANSparkMax(LiftConstants.INNER_HOOKS_PORT_1,
            MotorType.kBrushless);
    private RelativeEncoder innerHooksEncoder1;
    public boolean hitButton;


    private final CANSparkMax innerHooksMotor2 = new CANSparkMax(LiftConstants.INNER_HOOKS_PORT_2,
            MotorType.kBrushless);
    private RelativeEncoder innerHooksEncoder2;

    PIDController innerHooksPIDController = new PIDController(LiftConstants.INNER_HOOKS_P, LiftConstants.INNER_HOOKS_I,
            LiftConstants.INNER_HOOKS_D);

    DoubleSolenoid pneumatics_1 = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT, PneumaticsModuleType.REVPH,
            LiftConstants.LIFT_DS_CHANNEL_1_1, LiftConstants.LIFT_DS_CHANNEL_1_2);

    DoubleSolenoid pneumatics_2 = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT, PneumaticsModuleType.REVPH,
            LiftConstants.LIFT_DS_CHANNEL_2_1, LiftConstants.LIFT_DS_CHANNEL_2_2);

    public LiftSubsystem() {
       /*  ShuffleboardTab tab = Shuffleboard.getTab("Lift");
        tab.add("Inner Hooks Encoder Position", innerHooksEncoder1.getPosition());
        tab.add("Inner Hooks Encoder Velocity", innerHooksEncoder1.getVelocity());

        tab.add("Pneumatics 1 State", pneumatics_1.get());
        tab.add("Pneumatics 2 State", pneumatics_2.get());*/
    }

    // PREDICATES
    private boolean arePneumaticsExtended() {
        boolean pneumatics1check = false;
        boolean pneumatics2check = false;
        if (pneumatics_1.get() == DoubleSolenoid.Value.kForward) {
            pneumatics1check = true;
        }

        if (pneumatics_2.get() == DoubleSolenoid.Value.kForward) {
            pneumatics2check = true;
        }

        if (pneumatics1check && pneumatics2check) {
            return true;
        } else {
            return false;
        }
    }

    private boolean arePneumaticsRetracted() {
        boolean pneumatics1check = false;
        boolean pneumatics2check = false;
        if (pneumatics_1.get() == DoubleSolenoid.Value.kReverse) {
            pneumatics1check = true;
        }

        if (pneumatics_2.get() == DoubleSolenoid.Value.kReverse) {
            pneumatics2check = true;
        }

        if (pneumatics1check && pneumatics2check) {
            return true;
        } else {
            return false;
        }
    }

    // ENCODERS
    public void initEncoderandMotors() {
        innerHooksMotor2.follow(innerHooksMotor1);
        innerHooksEncoder1 = innerHooksMotor1.getEncoder();
        innerHooksEncoder2 = innerHooksMotor2.getEncoder();
    }

    public void resetEncoders() {
        innerHooksEncoder1.setPosition(0);
        innerHooksEncoder2.setPosition(0);
    }

    public double getInnerPos() {
        return innerHooksEncoder1.getPosition();
    }

    public void innerHooksSetOuput(double output) {
        innerHooksMotor2.follow(innerHooksMotor1);
        innerHooksMotor1.set(output);

    }

    public Object moveInnerHooks(double targetPos, double tolerance) {
        innerHooksPIDController.setTolerance(tolerance);
        double speed = innerHooksPIDController.calculate(innerHooksEncoder1.getPosition(), targetPos);
        innerHooksMotor2.follow(innerHooksMotor1);
        innerHooksMotor1.set(speed);
        return null;
    }

    // PNEUMATICS
    public void pnuematicsForward() {
        pneumatics_1.set(Value.kForward);
        pneumatics_2.set(Value.kForward);
    }

    public Object pnuematicsReverse() {
        pneumatics_1.set(Value.kReverse);
        pneumatics_2.set(Value.kReverse);
        return null;
    }

    public void pnuematicsOff() {
        pneumatics_1.set(Value.kOff);
        pneumatics_2.set(Value.kOff);
    }

    @Override
    public void close() throws Exception {
        innerHooksMotor1.close();
        innerHooksMotor2.close();
    }

}
