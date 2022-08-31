package frc.robot.subsystems;

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
    }

    public void executeClimbingPneumatics(boolean deploy) {
        toggleClimbingPneumatics(deploy);
    }

    public void endClimbingPneumatics(){
        retractClimbingPneumatics();
    }

    @Override
    public void close() {

    }
}
