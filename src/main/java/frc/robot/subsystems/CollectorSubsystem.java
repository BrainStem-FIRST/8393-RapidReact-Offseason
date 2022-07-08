package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CollectorSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  /*private Encoder encoder;
  CANSparkMax collectorMotor = new CANSparkMax(CollectorConstants.COLLECTOR_MOTOR_ID, MotorType.kBrushless);
  DoubleSolenoid collectorPneumatics = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_PORT, PneumaticsModuleType.REVPH,
  PneumaticsConstants.LIFT_DS_CHANNEL_, PneumaticsConstants)*/

  public CollectorSubsystem() {
  }

  public void initialize() {
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
  }

  public void goToPosition() {
    m_encoder.setPosition(0);
    while (m_encoder.getPosition() < 200) {
      m_motor.set(0.75);
      SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
      SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
    }
  }

  public void lowerIntake() {
    
  }
}
