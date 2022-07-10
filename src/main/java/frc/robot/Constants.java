// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DrivetrainSubsystem;

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


           public final class TransferConstants {
                   public static final int TRANSFER_MOTOR_PORT_ID = 12;
           }
           public final class ShooterConstants {
                
                   public static final int SHOOTER_1_MOTOR_PORT_ID = 14; // FIXME
                   public static final int SHOOTER_2_MOTOR_PORT_ID = 16; // FIXME
                   public static final int TURRET_MOTOR_PORT_ID = 18; // FIXME
                   public static final int ELEVATOR_MOTOR_PORT_ID = 20; // FIXME
                }

     

        public final class TransferConstants {
                public static final int TRANSFER_MOTOR_PORT_ID = 12;

        }

        public final class ShooterConstants {
                public static final int SHOOTER_1_MOTOR_PORT_ID = 14;
                public static final int SHOOTER_2_MOTOR_PORT_ID = 16;
                public static final int TURRET_MOTOR_PORT_ID = 18;
                public static final int ELEVATOR_MOTOR_PORT_ID = 20;
        }

        public static final class PnuematicsConstants {
                public static final int PNEUMATICS_PORT = 9;

                public static final int COMPRESSOR_MIN_PRESSURE = 90;
                public static final int COMPRESSOR_MAX_PRESSURE = 120;

        }

        public final class IntakeConstants {
                // SPARK ID
                public static final int INTAKE_MOTOR_ID = 10;
                public static final int INTAKE_PNEUMATICS_DISTANCE = 10; //FIXME
                public static final double INTAKE_MOTOR_SPEED = 0.5; //FIXME
                //solonoid ports
                public static final int INTAKE_PNEUMATICS_PORT = 10; //FIXME
                public static final int INTAKE_DS_CHANNEL_3_1 = 0; //FIXME 
                public static final int INTAKE_DS_CHANNEL_3_2 = 0; //FIXME
        }

        public final class LiftConstants {
                public static final int LIFT_DS_CHANNEL_1_1 = 0;
                public static final int LIFT_DS_CHANNEL_1_2 = 1;

                public static final int LIFT_DS_CHANNEL_2_1 = 3;
                public static final int LIFT_DS_CHANNEL_2_2 = 4;
                
        }

        // Mihir added this
        public final class DrivetrainConstants {

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

                // FRONT LEFT MODULE
                public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
                public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8;
                public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11;
                // front left steer offset CONVERTING TO RADIANS USING MATH.PI INSTEAD OF
                // TORADIANS CUZ IT DON'T WORK
                public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -(Math.PI / 180) * 46.6;

                // FRONT RIGHT MODULE
                public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
                public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
                public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
                // front right steer offset
                public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -(Math.PI / 180) * (166.5);

                // BACK LEFT MODULE
                public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
                public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
                public static final int BACK_LEFT_MODULE_STEER_ENCODER = 17;
                public static final double BACK_LEFT_MODULE_STEER_OFFSET = -(Math.PI / 180) * (135.5);

                // BACK RIGHT MODULE
                public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5;
                public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
                public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 15;
                // back right steer offset
                public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -(Math.PI / 180) * 151.9;// Math.toRadians(151.9);

        }

        // Mihir added this
        public static final class AutoConstants {
                // CONSTRAINTS
                public static final double MAX_ROBOT_VOLTAGE_AUTONOMOUS = 2.0;
                // MEASURED IN RADIANS PER SECOND SQUARED FOR AUTONOMOUS
                public static final double MAX_ANGULAR_ACCELERATION = ConstraintsConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                                / 16;
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                ConstraintsConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 10,
                                MAX_ANGULAR_ACCELERATION);
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kPXController = 1.5;
                public static final double kPYController = 1.5;
                public static final double kPThetaController = 3;
                public static final double kMaxSpeedMetersPerSecond = ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND;
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
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