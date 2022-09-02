package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.PnuematicsConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final DoubleSolenoid intakePneumatics = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT,
      PnuematicsConstants.PNEUMATICS_MODULE_TYPE,
      IntakeConstants.INTAKE_DS_CHANNEL_3_1, IntakeConstants.INTAKE_DS_CHANNEL_3_2);

  private boolean isIntakeMotorStopped() {
    return intakeMotor.get() == 0;
  }

  private boolean isIntakeMotorRunningAtRightSpeed() {
    return intakeMotor.get() < IntakeConstants.INTAKE_MOTOR_SPEED + IntakeConstants.INTAKE_MOTOR_SPEED_ERROR_ALLOWANCE
        && intakeMotor.get() > IntakeConstants.INTAKE_MOTOR_SPEED - IntakeConstants.INTAKE_MOTOR_SPEED_ERROR_ALLOWANCE;
  }

  private boolean isPneumaticsRetracted() {
    return intakePneumatics.get() == DoubleSolenoid.Value.kReverse;
  }

  private boolean isPneumaticsExtended() {
    return intakePneumatics.get() == DoubleSolenoid.Value.kForward;
  }

  private boolean isIntakeReadyToRetract() { // FIXME - More may need to be added
    return isPneumaticsExtended() && isIntakeMotorRunningAtRightSpeed();
  }

  private boolean isIntakeReadyToDeploy() { // FIXME - More may need to be added
    return isIntakeMotorStopped() && isPneumaticsRetracted();
  }

  public void startingCommands() {
    setOutput(0);
  }

  public void endingCommands() {
    if (isIntakeReadyToRetract()) {
      retractIntake();
    }
  }

  public void setOutput(double speed) {
    double intakeSpeed = IntakeConstants.INTAKE_MOTOR_REVERSED ? -speed : speed;
    intakeMotor.set(intakeSpeed);
  }

  public void lowerIntake() {
    intakePneumatics.set(Value.kForward);
  }

  public void raiseIntake() {
    intakePneumatics.set(Value.kReverse);
  }

  public void deployIntake(boolean reverse) {
    lowerIntake();
    int negative = reverse ? -1 : 1;
    setOutput(IntakeConstants.INTAKE_MOTOR_SPEED * negative);
  }

  public void retractIntake() {
   // raiseIntake();
    setOutput(0);
  }

  public void toggleIntake(boolean deploy, boolean reversed) {
    if (deploy) {
      deployIntake(reversed);
    } else {
      retractIntake();
    }
  }

  public void initializeIntake() {
    retractIntake();
  }

  public void executeIntake(boolean deploy, boolean reversed) {
    toggleIntake(deploy, reversed);
  }

  public void endIntake() {
    retractIntake();
  }

  @Override
  public void close() throws Exception {
    intakeMotor.close();
    intakePneumatics.close();
  }

}
