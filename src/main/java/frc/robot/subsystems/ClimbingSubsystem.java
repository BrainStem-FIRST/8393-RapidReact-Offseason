package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.PnuematicsConstants;

public class ClimbingSubsystem extends SubsystemBase implements AutoCloseable{
    private final DoubleSolenoid climbingPneumaticsLeft = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT,
            PnuematicsConstants.PNEUMATICS_MODULE_TYPE,
            ClimbingConstants.LEFT_CLIMBING_PNEUMATICS_FORWARD_CHANNEL,
            ClimbingConstants.LEFT_CLIMBING_PNEUMATICS_REVERSE_CHANNEL);
    private final DoubleSolenoid climbingPneumaticsRight = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT,
            PnuematicsConstants.PNEUMATICS_MODULE_TYPE,
            ClimbingConstants.RIGHT_CLIMBING_PNEUMATICS_FORWARD_CHANNEL,
            ClimbingConstants.RIGHT_CLIMBING_PNEUMATICS_REVERSE_CHANNEL);
    private final CANSparkMax climbingMotor1 = new CANSparkMax(ClimbingConstants.CLIMBING_MOTOR_1, MotorType.kBrushless);
    private final CANSparkMax climbingMotor2 = new CANSparkMax(ClimbingConstants.CLIMBING_MOTOR_2, MotorType.kBrushless);


    

    private void extendClimbingPneumatics() {
        climbingPneumaticsLeft.set(Value.kForward);
        climbingPneumaticsRight.set(Value.kForward);
    }

    private void retractClimbingPneumatics() {
        climbingPneumaticsLeft.set(Value.kReverse);
        climbingPneumaticsRight.set(Value.kReverse);
    }

    public void toggleClimbingPneumatics(boolean deploy) {
        if (deploy) {
            extendClimbingPneumatics();
        } else {
            retractClimbingPneumatics();
        }
    }
    

    public void initializeClimbingPneumatics(){
        retractClimbingPneumatics();
        setClimbingMotorPowers(0.0);
    }

    public void executeClimbingPneumatics(boolean deploy) {
        toggleClimbingPneumatics(deploy);
    }

    public void setClimbingMotorPowers(double speed){
        climbingMotor1.set(speed);
        climbingMotor2.follow(climbingMotor1);
    }

    public void executeClimbingMotors(double speed){
        setClimbingMotorPowers(speed);
    }

    public void endClimbingPneumatics(){
        retractClimbingPneumatics();
        setClimbingMotorPowers(0.0);
    }

    @Override
    public void close() {

    }
}