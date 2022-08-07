import static org.junit.Assert.*;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.subsystems.IntakeSubsystem;
import org.junit.Test;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import org.junit.*;

public class IntakeSubsystemTest {
   // IntakeSubsystem intakeSubsystem;
    CANSparkMax simMotor;
    
   // @Before
    //public void setup(){
    //   intakeSubsystem = new IntakeSubsystem();
      // simMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
   // }

    // this method will run after each test
    //public void shutdown() throws Exception {
    //  intakeSubsystem.close(); // destroy our intake object
   // }

    @Test
    public void worksWhenOpen() {
       
    }

    /*@Test
    public void testDeployIntake() {
        
    }

    @Test
    public void testEnd() {

    }

    @Test
    public void testInitialize() {

    }

    @Test
    public void testIsIntakeReadyToDeploy() {

    }

    @Test
    public void testIsIntakeReadyToRetract() {

    }

    @Test
    public void testLowerIntake() {

    }

    @Test
    public void testRaiseIntake() {

    }

    @Test
    public void testRetractIntake() {

    }

    @Test
    public void testSetOutput() {

    }

    @Test
    public void testToggleIntake() {

    }
    
    */
}
