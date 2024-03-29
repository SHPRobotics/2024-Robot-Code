// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
/*
 * MOTOR                    CANID
 * ------------------------ ------
 * FL Driving motor         11
 * FL Turning motor         10
 * FR Driving motor         15
 * FR Turning motor         14
 * RL Driving motor         13
 * RL Turning motor         12
 * RR Driving motor         17
 * RR Turning motor         16
 * 
 * Shooter L motor          8
 * Shooter R motor          9
 * 
 * Arm (Arm) motor     7
 * 
 * Intake motor             6
 * Limit Switch Channel channel 9
 */
  public static final boolean DEBUG = true;
  // OIConstants =================================================================================
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.08;  //0.05;
    public static final int kDriveControllerPort = 0;
  }

  // DriveConstants ===================================================================================
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
  }

  // ModuleConstants ===============================================================================
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762; // 3"
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  // ShooterConstants ===========================================================================
  public static final class ShooterConstants {

    public static final int kShooterLeftMotorCANId = 8;
    public static final int kShooterRightMotorCANId = 9;

    public static final IdleMode kShooterMotorIdleMode = IdleMode.kBrake;
    public static final int kShooterMotorCurrentLimit = 40; // amps
    
    public static final double kShooterSpeedOut = 1.0;
    public static final double kShooterSpeedIn = 0.3;
  }

  // ArmConstants ========================================================================
  public static final class ArmConstants {
    public static final int kArmMotorCANId = 7;
	  public static final double kArmSpeed = 0.25; // 0.2;
    public static final float kArmForwardLimit = 0;
    public static final float kArmReverseLimit = 0;
    public static final int limitSwitchChannel = 9;
    
    public static final double kArmAngleNeutral = 0;          // 
    public static final double kArmAngleSource = -20;      // 
    public static final double kArmAngleAmp = -47;       // 
    public static final double kArmAngleSpeaker = -3;    //
    public static final double kArmAngleSideSpeaker = -2; // 
    //best fit equation for (angle, encoderValue) relationship: V = 
    
    //public static final double kInclinerAngleGroundIntake =  0; // change it to kArmAngleNeutral

    public static final int kArmMotorCurrentLimit = 40;     // amps    
    public static final double kP = 1;
    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;
    private static final double kWheelDiameterInches = 6;
    private static final double kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kArmOffsetRads = 0.5;

    public static final double kSVolts = 1.0;
    public static final double kGVolts = 1.0;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;
  }
  
  // GroundIntakeConstants ========================================================================
  public static final class GroundIntakeConstants {
    public static final int kRightGroundIntakeMotorCANId = 6;
    public static final int kLeftGroundIntakeMotorCANId = 5;
    public static final int kTopGroundIntakeMotorCANId = 4;
    public static final double kGroundIntakeSpeed = 1;
    public static final double kTopGroundIntakeSpeed = 0.35;
    public static final IdleMode kGroundIntakeMotorIdleMode = IdleMode.kBrake;
    public static final int kGroundIntakeMotorCurrentLimit =40; //amps
  }

  // AutoConstants ==============================================================================
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    
    public static final double kAutoDriveSpeed = 0.4; //0.25;  // speed of robot in Autonomous mode
    public static final double kAutoOmniDirectionSpeed = 2.25; //strafing speed of robot when using omnidirection method in autonomous mode in meters per second
    public static final double kAutoTurnSpeed = 0.25;  // speed of robot in Autonomous mode
    //public static final double kAutoArmSpeed = 0.2; // speed of arm in Autonomous mode

  }

  // NeoMotorConstants =========================================================================
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  
}
