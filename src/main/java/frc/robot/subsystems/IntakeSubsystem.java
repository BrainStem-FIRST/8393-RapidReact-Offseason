package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import frc.robot.commands.DefaultDriveCommand;
//import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.PnuematicsConstants;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final DoubleSolenoid intakePneumatics = new DoubleSolenoid(PnuematicsConstants.PNEUMATICS_PORT,
      PneumaticsModuleType.REVPH,
      IntakeConstants.INTAKE_DS_CHANNEL_3_1, IntakeConstants.INTAKE_DS_CHANNEL_3_2);

  // PREDICATES FOR INTAKE SUBSYSTEM
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

  public boolean isIntakeReadyToRetract() { //FIXME
    return isPneumaticsExtended() && isIntakeMotorRunningAtRightSpeed();
  }

  public boolean isIntakeReadyToDeploy() { //FIXME
    return isIntakeMotorStopped() && isPneumaticsRetracted();
  }

  public void initialize() {
    intakeMotor.restoreFactoryDefaults();
    setOutput(0);
  }

  public void end() {
    // HOW DO YOU RETURN WITH IF IN FRONT?
    if (isIntakeReadyToRetract()) {
      retractIntake();
    }
  }

  public void setOutput(double speed) {
    intakeMotor.set(speed);
  }

  public void lowerIntake() {
    intakePneumatics.set(Value.kForward);
  }

  public void raiseIntake() {
    intakePneumatics.set(Value.kReverse);
  }

  public void deployIntake() {
    lowerIntake();
    setOutput(IntakeConstants.INTAKE_MOTOR_SPEED);
  }

  public void retractIntake() {
    raiseIntake();
    setOutput(0);
  }

  public void toggleIntake(boolean deploy) {
    if (isIntakeReadyToDeploy() && deploy) {
      deployIntake();
    } else if(!deploy && isIntakeReadyToRetract()) {
      retractIntake();
    }
  }
}
