// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static final class Driver1ControllerConstants {
                public static final int CONTROLLER_PORT = 0;
                public static final double CONTROLLER_DEADZONE = 0.05; //FIXME
        }

        public static final class Driver2ControllerConstants {
                public static final int CONTROLLER_PORT = 1;
                public static final double CONTROLLER_DEADZONE = 0.05; //FIXME
        }

        public static final class TransferConstants {
                public static final int TRANSFER_MOTOR_PORT_ID = 12; // FIXME
                public static final double TRANSFER_MOTOR_SPEED = 0.5; // FIXME
        }

        public static final class ShooterConstants {
                public static final double PROPORTIONAL = 1.17; // FIXME
                public static final double INTREGRAL = 0.0017; // FIXME
                public static final double DERIVATIVE = 0; // FIXME
                public static final double SET_PID_LOCATION = 0.5; // FIXME
                public static final int SHOOTER_1_MOTOR_PORT_ID = 14; // FIXME
                public static final int SHOOTER_2_MOTOR_PORT_ID = 16; // FIXME
                public static final int TURRET_MOTOR_PORT_ID = 18; // FIXME
                public static final int ELEVATOR_MOTOR_PORT_ID = 20; // FIXME
        }

        public static final class PnuematicsConstants {
                public static final int PNEUMATICS_PORT = 9;
                public static final int COMPRESSOR_MIN_PRESSURE = 90;
                public static final int COMPRESSOR_MAX_PRESSURE = 120;
        }

        public static final class IntakeConstants {
                // SPARK ID
                public static final int INTAKE_MOTOR_ID = 10; // FIXME
                public static final int INTAKE_PNEUMATICS_DISTANCE = 10; // FIXME
                public static final double INTAKE_MOTOR_SPEED = 0.5; // FIXME
                // solonoid ports
                public static final int INTAKE_PNEUMATICS_PORT = 10; // FIXME
                public static final int INTAKE_DS_CHANNEL_3_1 = 0; // FIXME
                public static final int INTAKE_DS_CHANNEL_3_2 = 0; // FIXME
                public static final double INTAKE_MOTOR_SPEED_ERROR_ALLOWANCE = 0.15; // FIXME
        }

        public static final class LiftConstants {
                public static final int INNER_HOOKS_PORT = 1;
                public static final int OUTER_HOOKS_PORT = 1;

                public static final int LIFT_DS_CHANNEL_1_1 = 0;
                public static final int LIFT_DS_CHANNEL_1_2 = 1;

                public static final int LIFT_DS_CHANNEL_2_1 = 3;
                public static final int LIFT_DS_CHANNEL_2_2 = 4;

        }

        public static final class SwerveModuleConstants {
                public static final double WHEEL_DIAMETER_METERS = 0.10033;
                public static final double DRIVE_MOTOR_GEAR_RATIO = (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0);
                public static final double TURNING_MOTOR_GEAR_RATIO = (15.0 / 32.0) * (10.0 / 60.0) * 0.66;
                public static final double DRIVE_ENCODER_TICKS_TO_METERS = (DRIVE_MOTOR_GEAR_RATIO * Math.PI
                                * WHEEL_DIAMETER_METERS) / 2048;
                public static final double TURNING_ENCODER_TICKS_TO_RADIANS = (TURNING_MOTOR_GEAR_RATIO * Math.PI * 2)
                                / 2048;
                public static final double DRIVE_ENCODER_TICKS_TO_METERS_PER_SECOND = (DRIVE_ENCODER_TICKS_TO_METERS
                                / 60) / 2048;
                public static final double TURNING_ENCODER_TICKS_TO_METERS_PER_SECOND = (TURNING_ENCODER_TICKS_TO_RADIANS
                                / 60) / 2048;

                // PID
                public static final double PROPORTIONAL = 0.5; // FIXME
                public static final double INTEGRAL = 0;
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
                // KINEMATICS
                public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                                new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2, -DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2, DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2, -DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2, DRIVETRAIN_TRACKWIDTH_METERS / 2));

                // FRONT LEFT MODULE
                public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
                public static final int FRONT_LEFT_MODULE_TURNING_MOTOR = 8;
                public static final int FRONT_LEFT_MODULE_TURNING_ABSOLUTE_ENCODER = 11;
                public static final boolean FRONT_LEFT_TURNING_ABSOLUTE_ENCODER_REVERSED = false; // FIXME
                public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false; // FIXME
                // front left TURNING offset
                public static final double FRONT_LEFT_MODULE_TURNING_OFFSET = -Math.toRadians(46.6);

                // FRONT RIGHT MODULE
                public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
                public static final int FRONT_RIGHT_MODULE_TURNING_MOTOR = 2;
                public static final int FRONT_RIGHT_MODULE_TURNING_ABSOLUTE_ENCODER = 13;
                public static final boolean FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER_REVERSED = false; // FIXME
                public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false; // FIXME
                // front right TURNING offset
                public static final double FRONT_RIGHT_MODULE_TURNING_OFFSET = -Math.toRadians(166.5);

                // BACK LEFT MODULE
                public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
                public static final int BACK_LEFT_MODULE_TURNING_MOTOR = 6;
                public static final int BACK_LEFT_MODULE_TURNING_ABSOLUTE_ENCODER = 17;
                public static final boolean BACK_LEFT_TURNING_ABSOLUTE_ENCODER_REVERSED = false; // FIXME
                public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false; // FIXME
                // back left TURNING offset
                public static final double BACK_LEFT_MODULE_TURNING_OFFSET = -Math.toRadians(135.5);

                // BACK RIGHT MODULE
                public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5;
                public static final int BACK_RIGHT_MODULE_TURNING_MOTOR = 4;
                public static final int BACK_RIGHT_MODULE_TURNING_ABSOLUTE_ENCODER = 15;
                public static final boolean BACK_RIGHT_TURNING_ABSOLUTE_ENCODER_REVERSED = false; // FIXME
                public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false; // FIXME
                // back right TURNING offset
                public static final double BACK_RIGHT_MODULE_TURNING_OFFSET = -Math.toRadians(151.9);

        }

        // Mihir added this
        public static final class AutoConstants {
                // CONSTRAINTS
                public static final double MAX_ROBOT_VOLTAGE_AUTONOMOUS = 2.0;
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
                public static final SwerveDriveKinematics autoDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2,
                                                -DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2,
                                                DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(-DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2,
                                                -DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(-DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2,
                                                DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2));

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
                                SdsModuleConfigurations.MK4_L4.getWheelDiameter() * Math.PI;
                // ABSOLUTE MAXIMUM ANGULAR VELOCITY OF ROBOT
                public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND
                                /
                                Math.hypot(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                                DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
        }
}