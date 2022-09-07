package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

        public static final class JoystickConstants {

                public static final int LEFT_STICK_X_AXIS = 0; 
                public static final int LEFT_STICK_Y_AXIS = 1;
                public static final int LEFT_TRIGGER = 2;
                public static final int RIGHT_TRIGGER = 3;
                public static final int RIGHT_STICK_Y_AXIS = 5;
                public static final int RIGHT_STICK_X_AXIS = 4;
                public static final int A_BUTTON = 1;
                public static final int B_BUTTON = 2;
                public static final int X_BUTTON = 3;
                public static final int Y_BUTTON = 4;
                public static final int LEFT_BUMPER = 5;                
                public static final int RIGHT_BUMPER = 7;
                public static final int BACK_BUTTON = 8;
                public static final int START_BUTTON = 9;
                public static final int LEFT_JOYSTICK_BUTTON = 10;
                public static final int RIGHT_JOYSTICK_BUTTON = 11;
        }

        public static final class Driver1ControllerConstants {
                public static final int CONTROLLER_PORT = 0;
                public static final double CONTROLLER_DEADZONE = 0.05;
                public static final int DRIVING_EXPONENT = 3;
                public static final double TRIGGER_ACTIVATION_THRESHOLD = 0.05;
        }

        public static final class Driver2ControllerConstants {
                public static final int CONTROLLER_PORT = 1;
                public static final double CONTROLLER_DEADZONE = 0.05; 
                public static final int DRIVING_EXPONENT = 3;
                public static final double TRIGGER_ACTIVATION_THRESHOLD = 0.05;
        }

        public static final class TransferConstants {
                public static final int TRANSFER_MOTOR_PORT_ID = 18; 
                public static final boolean TRANSFER_MOTOR_REVERSED = false;
                public static final double TRANSFER_MOTOR_SPEED = 1; 
                public static final double TRANSFER_PROPORTIONAL = 1.17; // FIXME
                public static final double TRANSFER_INTEGRAL = 0.0017; // FIXME
                public static final double TRANSFER_DERIVATIVE = 0; // FIXME
        }

        public final class ShooterConstants {
                // TURRET PID VALUES
                public static final double TURRET_PROPORTIONAL = 1.17; // FIXME
                public static final double TURRET_INTREGRAL = 0.0017; // FIXME
                public static final double TURRET_DERIVATIVE = 0; // FIXME
                public static final double TURRET_PID_TOLERANCE = 3; // FIXME
                // ELEVATOR PID VALUES
                public static final double ELEVATOR_PROPORTIONAL = 1.17; // FIXME
                public static final double ELEVATOR_INTEGRAL = 0.0017; // FIXME
                public static final double ELEVATOR_DERIVATIVE = 0; // FIXME
                public static final double ELEVATOR_PID_TOLERANCE = 3; // FIXME
                // SHOOTER PID VALUES
                public static final double SHOOTER_PROPORTIONAL = 1.17; // FIXME
                public static final double SHOOTER_INTEGRAL = 0.0017; // FIXME
                public static final double SHOOTER_DERIVATIVE = 0; // FIXME
                public static final double SHOOTER_PID_TOLERANCE = 3; // FIXME

                public static final int SHOOTER_1_MOTOR_PORT_ID = 24;
                public static final int SHOOTER_2_MOTOR_PORT_ID = 21;
                public static final int TURRET_MOTOR_PORT_ID = 26; // positive turns right, negative turns left

                public static final boolean SHOOTING_MOTORS_REVERSED = false;
                public static final boolean ELEVATOR_MOTOR_REVERSED = false;
                public static final boolean TURRET_MOTOR_REVERSED = false;
                public static final double SHOOTING_MOTORS_SPEED = 1;
                public static final double ELEVATOR_MOTOR_SPEED = 0.3;

                public static final int ROTATION_TO_TICKS = 28;
        }

        public static final class PnuematicsConstants {
                public static final int PNEUMATICS_PORT = 9;
                public static final int COMPRESSOR_MIN_PRESSURE = 100;
                public static final int COMPRESSOR_MAX_PRESSURE = 120;
                public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
        }

        public static final class IntakeConstants {
                public static final boolean INTAKE_MOTOR_REVERSED = true;
                public static final int INTAKE_MOTOR_ID = 20;
                public static final double INTAKE_MOTOR_SPEED = 1; 
                // solonoid ports
                public static final int INTAKE_PNEUMATICS_PORT = 9; // FIXME
                public static final int INTAKE_DS_CHANNEL_3_1 = 1; // FIXME 1
                public static final int INTAKE_DS_CHANNEL_3_2 = 6; // FIXME 6
                public static final double INTAKE_MOTOR_SPEED_ERROR_ALLOWANCE = 0.15;

        }

        public static final class ClimbingConstants {
                public static final int LEFT_CLIMBING_PNEUMATICS_FORWARD_CHANNEL = 2;
                public static final int LEFT_CLIMBING_PNEUMATICS_REVERSE_CHANNEL = 5;
                public static final int RIGHT_CLIMBING_PNEUMATICS_FORWARD_CHANNEL = 0;
                public static final int RIGHT_CLIMBING_PNEUMATICS_REVERSE_CHANNEL = 7;
                public static final int CLIMBING_MOTOR_1 = 22; //FIXME
                public static final int CLIMBING_MOTOR_2 = 19; //FIXME
                public static final double CLIMBING_MOTOR_SPEEDS = 0.5; //FIXME
                public static final boolean REVERSE_CLIMBING_MOTORS = false;
        }

        public static final class SwerveModuleConstants {
                public static final double WHEEL_DIAMETER_METERS = 0.10033;
                public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 8.14; // (16.0 / 48.0) * (28.0 / 16.0) * (15.0 /
                                                                              // 45.0)
                public static final double TURNING_MOTOR_GEAR_RATIO = 1 / (150 / 7); // (15.0 / 32.0) * (10.0 / 60.0) *
                                                                                     // 0.6666666
                public static final double DRIVE_ENCODER_TICKS_TO_METERS = (DRIVE_MOTOR_GEAR_RATIO * Math.PI
                                * WHEEL_DIAMETER_METERS) / 2048;
                public static final double TURNING_ENCODER_TICKS_TO_RADIANS = (TURNING_MOTOR_GEAR_RATIO * Math.PI * 2)
                                / 2048;
                public static final double DRIVE_ENCODER_TICKS_TO_METERS_PER_SECOND = (DRIVE_ENCODER_TICKS_TO_METERS
                                / 60) / 2048;
                public static final double TURNING_ENCODER_TICKS_TO_METERS_PER_SECOND = (TURNING_ENCODER_TICKS_TO_RADIANS
                                / 60) / 2048;

                // PID
                public static final double PROPORTIONAL = 1.17; // FIXME
                public static final double INTEGRAL = 0.0017;
                public static final double DERIVATIVE = 0;
        }

        public static final class DrivetrainConstants {

                /**
                 * The left-to-right distance between the drivetrain wheels
                 *
                 * Should be measured from center to center.
                 */
                public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.52705;
                /**
                 * The front-to-back distance between the drivetrain wheels.
                 *
                 * Should be measured from center to center.
                 */
                public static final double DRIVETRAIN_WHEELBASE_METERS = 0.73025;
                //LIMITERS
                public static final double DRIVETRAIN_SPEED_LIMITER = 0.5;
                public static final double TURNING_LIMITER = 0.9;

                // KINEMATICS
                public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                                new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2, -DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2, DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2, -DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2, DRIVETRAIN_TRACKWIDTH_METERS / 2));

                // FRONT LEFT MODULE
                public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
                public static final boolean FRONT_LEFT_DRIVE_MOTOR_REVERSED = false;
                public static final int FRONT_LEFT_MODULE_TURNING_MOTOR = 8;
                public static final int FRONT_LEFT_MODULE_TURNING_ABSOLUTE_ENCODER = 11;
                public static final boolean FRONT_LEFT_TURNING_ABSOLUTE_ENCODER_REVERSED = false; // FIXME
                public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = true; // FIXME
                // front left TURNING offset
                public static final double FRONT_LEFT_MODULE_TURNING_OFFSET = -Math.toRadians(46.6); // 46.6
                // FRONT RIGHT MODULE
                public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
                public static final boolean FRONT_RIGHT_DRIVE_MOTOR_REVERSED = false;
                public static final int FRONT_RIGHT_MODULE_TURNING_MOTOR = 2;
                public static final int FRONT_RIGHT_MODULE_TURNING_ABSOLUTE_ENCODER = 13;
                public static final boolean FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER_REVERSED = false; // FIXME
                public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false; // FIXME
                // front right TURNING offset
                public static final double FRONT_RIGHT_MODULE_TURNING_OFFSET = -Math.toRadians(166.5); // 166.5


                // BACK LEFT MODULE
                public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
                public static final boolean BACK_LEFT_DRIVE_MOTOR_REVERSED = false;
                public static final int BACK_LEFT_MODULE_TURNING_MOTOR = 6;
                public static final int BACK_LEFT_MODULE_TURNING_ABSOLUTE_ENCODER = 17;
                public static final boolean BACK_LEFT_TURNING_ABSOLUTE_ENCODER_REVERSED = false; // FIXME
                public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false; // FIXME
                // back left TURNING offset
                public static final double BACK_LEFT_MODULE_TURNING_OFFSET = -Math.toRadians(135.5); // 135.5
                // BACK RIGHT MODULE
                public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5;
                public static final boolean BACK_RIGHT_DRIVE_MOTOR_REVERSED = false;
                public static final int BACK_RIGHT_MODULE_TURNING_MOTOR = 4;
                public static final int BACK_RIGHT_MODULE_TURNING_ABSOLUTE_ENCODER = 15;
                public static final boolean BACK_RIGHT_TURNING_ABSOLUTE_ENCODER_REVERSED = false; // FIXME
                public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false; // FIXME
                // back right TURNING offset
                public static final double BACK_RIGHT_MODULE_TURNING_OFFSET = -Math.toRadians(151.9); // 151.9

        }

        public static final class AutoConstants {
                // CONSTRAINTS
                public static final double MAX_ROBOT_VOLTAGE_AUTONOMOUS = 12.0;
                // MEASURED IN RADIANS PER SECOND SQUARED FOR AUTONOMOUS
                public static final double MAX_ANGULAR_ACCELERATION = ConstraintsConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                                / 16;
                public static final TrapezoidProfile.Constraints autoThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                ConstraintsConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 10,
                                MAX_ANGULAR_ACCELERATION);
                public static final double autoMaxAccelerationMetersPerSecondSquared = 3;
                public static final double autoXController = 1.5;
                public static final double autoYController = 1.5;
                public static final double autoThetaController = 3;
                public static final double autoMaxSpeedMetersPerSecond = ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND;
                 }

                // Mihir added this
                public static final class ConstraintsConstants {
                        // ABSOLUTE MAXIMUM VOLTAGE OF ROBOT
                        public static final double MAX_ROBOT_VOLTAGE = 12.0;
                        // ABSOLUTE MAXIUMUM VELOCITY OF ROBOT
                        // The formula for calculating the theoretical maximum velocity is:
                        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
                        // pi
                        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                                        SdsModuleConfigurations.MK4_L4.getDriveReduction() *
                                        SdsModuleConfigurations.MK4_L4.getWheelDiameter() * Math.PI*1.1;
                        // ABSOLUTE MAXIMUM ANGULAR VELOCITY OF ROBOT
                        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND
                                        /
                                        Math.hypot(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
                        public static final int CAN_SPARK_MAX_MAXIMUM_RPM = 5700; // FIXME
                }
        }
