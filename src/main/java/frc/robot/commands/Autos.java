// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  // DriveDistanceAuto(): drive forward / backward a specified distance then stop
  public static Command DriveDistanceAuto(DriveSubsystem driveSubsystem, boolean driveReversed, double distanceMeters){
    return new FunctionalCommand(
      // onInit: reset encoders on command start
        driveSubsystem::resetEncoders,

      // onExecute: drive forward (if driveReversed = false) or reverse (if driveReversed = true) at an angle while command is executing
      // robot drives in robot-centric mode (fieldRelative = false)
      () -> driveSubsystem.drive(AutoConstants.kAutoDriveSpeed * (driveReversed ? -1 : 1) , 
                                 0,  
                             0, false, true), 

      // onEnd: stop driving at the end of command
      interrupt -> driveSubsystem.setX(), 

      // isFinished: End the command when the robot's driven distance exceeds the desired value
      // for some reason, the real traveled distance is off by 0.5 meter !
      () -> driveSubsystem.getAverageEncoderDistance() >= (distanceMeters),
      
      // require the drive subsystem
      driveSubsystem);
  }

  // StrafeDistanceAuto(): strafe right / left a specified distance then stop
  public static Command StrafeDistanceAuto(DriveSubsystem driveSubsystem, boolean strafeLeft, double distanceMeters){
    return new FunctionalCommand(
      // onInit: reset encoders on command start
      driveSubsystem::resetEncoders,

      // onExecute: strafe left (if strafeLeft = true) or right (if strafeLeft = false) while command is executing
      // robot drives in robot-centric mode (fieldRelative = false)
      () -> driveSubsystem.drive(0,
                                 AutoConstants.kAutoDriveSpeed * (strafeLeft ? 1 : -1) , 
                             0, false, true), 

      // onEnd: stop driving at the end of command
      interrupt -> driveSubsystem.setX(), 

      // isFinished: End the command when the robot's driven distance exceeds the desired value
      // for some reason, the real traveled distance is off by 0.5 meter !
      () -> driveSubsystem.getAverageEncoderDistance() >= (distanceMeters ),
      
      // require the drive subsystem
      driveSubsystem);
  }

  // DriveAngleDistanceAuto(): drive forward / backward at an angle a specified distance then stop
  public static Command DriveAngleDistanceAuto(DriveSubsystem driveSubsystem, boolean driveReversed, double distanceMeters, double driveAngleDeg){
    return new FunctionalCommand(
      // onInit: reset encoders on command start
      driveSubsystem::resetEncoders,

      // onExecute: drive forward (if driveReversed = false) or reverse (if driveReversed = true) at an angle while command is executing
      // robot drives in robot-centric mode (fieldRelative = false)
/*
      () -> driveSubsystem.drive(AutoConstants.kAutoDriveSpeed * (driveReversed ? -1 : 1) * Math.cos(Units.degreesToRadians(driveAngleDeg)), 
                                 AutoConstants.kAutoDriveSpeed * (driveReversed ? -1 : 1) * Math.sin(Units.degreesToRadians(driveAngleDeg)),  
                             0, false, true), 
*/
      ()-> driveSubsystem.omniDirectionStrafe(AutoConstants.kAutoOmniDirectionSpeed * (driveReversed ? -1 : 1), driveAngleDeg),
      // onEnd: stop driving at the end of command
      //interrupt -> driveSubsystem.setX(), 
      interrupt -> driveSubsystem.stop(), 

      // isFinished: End the command when the robot's driven distance exceeds the desired value
      // for some reason, the real traveled distance is off by 0.5 meter !
      () -> driveSubsystem.getAverageEncoderDistance() >= (distanceMeters),
      
      // require the drive subsystem
      driveSubsystem);
  }

  // RotateRobotAuto(): rotate the robot at a specified angle then stop
  public static Command RotateRobotAuto(DriveSubsystem driveSubsystem, boolean turnClockwise, double angleDeg){
    return new FunctionalCommand(
      // onInit: reset encoders on command start
      driveSubsystem::zeroHeading,

      // onExecute: strafe left (if strafeLeft = true) or right (if strafeLeft = false) while command is executing
      // robot drives in robot-centric mode (fieldRelative = false)
      () -> driveSubsystem.drive(0,
                                 0 , 
                             AutoConstants.kAutoTurnSpeed * (turnClockwise ? -1 : 1), false, true), 

      // onEnd: stop driving at the end of command
      interrupt -> driveSubsystem.setX(), 

      // isFinished: End the command when the robot's driven distance exceeds the desired value
      () -> Math.abs(driveSubsystem.getGyro().getAngle()) >= Math.abs((angleDeg)),
      
      // require the drive subsystem
      driveSubsystem);
  }
  
  /*
  // Red 1:
            -shoot note into speaker
            -drive reverse
            -turn clockwise
            -drive reverse->(parallel)->run ground intake motor 
            -drive foward
            -turn counterclockwise
            -drive foward
            -shoot note into speaker
*/
  public static Command turnTwice (DriveSubsystem driveSubsystem){
    return Commands.sequence(
      RotateRobotAuto(driveSubsystem, false, 180),
      new WaitCommand(1),
      RotateRobotAuto(driveSubsystem, true, 180)
    );
  }
  public static Command red1 (DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, GroundIntakeSubsystem groundIntakeSubsystem){
    return Commands.sequence(
      //start with arm down
      //new ArmDown(armSubsystem),
      // set arm to speaker position
      new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker),
      new WaitCommand(1),
      // -shoot note into speaker
      shooterSubsystem.ShooterShootNoteOutCmd(),
      new WaitCommand(1),
      //stop shooter
      shooterSubsystem.ShooterStopCmd(),
      //Bring arm down while driving back
      new ParallelCommandGroup(
        new ArmDown(armSubsystem),
        DriveDistanceAuto(driveSubsystem, true, 0.4)),      
      // -turn clockwise
      RotateRobotAuto(driveSubsystem, true, 45),
      //drive back while intaking
      new ParallelCommandGroup(
        DriveDistanceAuto(driveSubsystem, true, 1.6),
        groundIntakeSubsystem.GroundIntakeFeedNoteInCmd()),
      new WaitCommand(1),
      //stop Ground intake
      groundIntakeSubsystem.GroundIntakeStopCmd(),
      //drive foward while setting speaker angle
      new ParallelCommandGroup(
        DriveDistanceAuto(driveSubsystem, false, 1.6),
        new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker)),
      //turn counterclockwise
      RotateRobotAuto(driveSubsystem, false, 45) ,
      //drive foward
      DriveDistanceAuto(driveSubsystem, false, 0.6),
      //wait to adjust
      new WaitCommand(0.3),
      //shoot note
      shooterSubsystem.ShooterShootNoteOutCmd(),
      new WaitCommand(1),
      //stop shooter
      shooterSubsystem.ShooterStopCmd(),
      //bring arm down while driving back
      new ParallelCommandGroup(
        new ArmDown(armSubsystem),
        DriveDistanceAuto(driveSubsystem, true, 0.4)),      
      // -turn clockwise
      RotateRobotAuto(driveSubsystem, true, 45),
      //drive back behind line
      DriveDistanceAuto(driveSubsystem, true, 1.6)
    );
  }

  public static Command red2 (DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, GroundIntakeSubsystem groundIntakeSubsystem) {
    return Commands.sequence(
   /* //set arm zero
    new ArmDown(armSubsystem),*/
    //set arm to speaker position
    new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker),
    new WaitCommand(1),
    //shoot note
    shooterSubsystem.ShooterShootNoteOutCmd(),
    //wait
    new WaitCommand(1),
    //stop shooter
    shooterSubsystem.ShooterStopCmd(),
    //bring arm to zero
    new ArmDown(armSubsystem),
    //drive back to note while intaking
    new ParallelCommandGroup(
      DriveDistanceAuto(driveSubsystem, true, 1.5),
      groundIntakeSubsystem.GroundIntakeFeedNoteInCmd()), 
    //wait
    new WaitCommand(1),
    //stop ground intake
    groundIntakeSubsystem.GroundIntakeStopCmd(), 
    //drive foward while setting arm angle
    new ParallelCommandGroup(
      DriveDistanceAuto(driveSubsystem, false, 1.6),
      new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker)),
    //shoot note
    shooterSubsystem.ShooterShootNoteOutCmd(),
    //wait
    new WaitCommand(1),
    //stop shooter
    shooterSubsystem.ShooterStopCmd(),
    //drive backwards behind the line
    DriveDistanceAuto(driveSubsystem, true, 2)  
    );
  }
  public static Command fullRed2 (DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, GroundIntakeSubsystem groundIntakeSubsystem){
    return Commands.sequence(
    //set arm zero
    new ArmDown(armSubsystem),
    //set arm to speaker position
    new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker),
    new WaitCommand(1),
    //shoot note
    shooterSubsystem.ShooterShootNoteOutCmd(),
    //waits
    new WaitCommand(.5),
    //stop shooter
    shooterSubsystem.ShooterStopCmd(),
    //drive back to note while intaking
    new ParallelCommandGroup(
      //bring arm to zero
      new ArmDown(armSubsystem), 
      DriveDistanceAuto(driveSubsystem, true, 1.5),
      groundIntakeSubsystem.GroundIntakeFeedNoteInCmd()), 
    //wait
    new WaitCommand(0.75),
    //stop ground intake
    groundIntakeSubsystem.GroundIntakeStopCmd(), 
    //drive foward while setting arm angle
    new ParallelCommandGroup(
      DriveDistanceAuto(driveSubsystem, false, 1.55),
      new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker)),
    //wait to adjust
    new WaitCommand(.3),
    //shoot note
    shooterSubsystem.ShooterShootNoteOutCmd(),
    //wait
    new WaitCommand(.5),
    //stop shooter
    shooterSubsystem.ShooterStopCmd(),
    //position behind note by strafing at an angle
    // beginning of 3rd note -- make sure to mirror
    DriveAngleDistanceAuto(driveSubsystem, true, 1.5, 60),
    //drive back while intaking
    new ParallelCommandGroup(
      //bring arm down
      new ArmDown(armSubsystem),
      DriveDistanceAuto(driveSubsystem, true, 1),
      groundIntakeSubsystem.GroundIntakeFeedNoteInCmd()),
    //wait
    new WaitCommand(.75),
    //stop ground intake
    groundIntakeSubsystem.GroundIntakeStopCmd(),
    //drive foward while set speaker angle
    new ParallelCommandGroup(
      DriveAngleDistanceAuto(driveSubsystem, false, 2.3, 42.5),
      new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker)),
    //wait to adjust
    new WaitCommand(0.3),
    //shoot note
    shooterSubsystem.ShooterShootNoteOutCmd(),
    //wait
    new WaitCommand(.5),
    //stop shooter
    shooterSubsystem.ShooterStopCmd(),

    // 4th note -- mirroring 3rd ----------------
    DriveAngleDistanceAuto(driveSubsystem, true, 1.5, -60),
    //drive back while intaking
    new ParallelCommandGroup(
      //bring arm down
      new ArmDown(armSubsystem),
      DriveDistanceAuto(driveSubsystem, true, 1),
      groundIntakeSubsystem.GroundIntakeFeedNoteInCmd()),
    //wait
    new WaitCommand(0.75),
    //stop ground intake
    groundIntakeSubsystem.GroundIntakeStopCmd(),
    //drive foward while set speaker angle
    new ParallelCommandGroup(
      DriveAngleDistanceAuto(driveSubsystem, false, 2.3, -37.5),
      new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker)),
    //wait to adjust
    new WaitCommand(0.3),
    //shoot note
    shooterSubsystem.ShooterShootNoteOutCmd(),
    //wait
    new WaitCommand(0.5),
    //stop shooter
    shooterSubsystem.ShooterStopCmd(),
    //drive behind line
    DriveDistanceAuto(driveSubsystem, true, 2)
    );
  }



public static Command red3 (DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, GroundIntakeSubsystem groundIntakeSubsystem){
    return Commands.sequence(
      // set arm to speaker position
      new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker),
      new WaitCommand(1),
      // -shoot note into speaker
      shooterSubsystem.ShooterShootNoteOutCmd(),
      new WaitCommand(1),
      //stop shooter
      shooterSubsystem.ShooterStopCmd(),
      //Bring arm down
      new ArmDown(armSubsystem),
      // -drive back
      DriveDistanceAuto(driveSubsystem, true, .5),
      // -turn clockwise
      RotateRobotAuto(driveSubsystem, true, -45),
      //drive back while intaking
      new ParallelCommandGroup(
        DriveDistanceAuto(driveSubsystem, true, 1),
        groundIntakeSubsystem.GroundIntakeFeedNoteInCmd()),
      new WaitCommand(1),
      //stop Ground intake
      groundIntakeSubsystem.GroundIntakeStopCmd(),
      //drive foward
      DriveDistanceAuto(driveSubsystem, false, .5),
      //turn counterclockwise
      RotateRobotAuto(driveSubsystem, false, 45)/* ,
      //drive foward
      DriveDistanceAuto(driveSubsystem, false, .6),
      //set speaker angle
      new ArmSetAngle(armSubsystem, ArmConstants.kArmAngleSpeaker),
      new WaitCommand(1),
      //shoot note
      shooterSubsystem.ShooterShootNoteOutCmd(),
      new WaitCommand(1),
      //stop shooter
      shooterSubsystem.ShooterStopCmd()*/
    );
  }







  // 2. driveAlongPathAuto(): drive robot along a pre-defined path
  public static Command driveAlongPathAuto(DriveSubsystem driveSubsystem) {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, false, false));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
