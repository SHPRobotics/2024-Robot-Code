// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InclinerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.Constants.GroundIntakeConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // Shooter subsystem
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // Incliner subsystem
  private final InclinerSubsystem m_incliner = new InclinerSubsystem();
  // Ground Intake Subsystem
  private final GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  // The driver's controller
  
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriveControllerPort); //Added Command xbox controller to be able to use built in trigger commands
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort); //Added Command xbox controller to be able to use built in trigger commands

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
            //To change translation from joystick to D-pad, refer to GenericHID API for getPov() method
            //Test following code, rpelacing both lines 66 & 67: 
            //-MathUtil.applyDeadband(m_driverController.getPov(), OIConstants.kDriveDeadband)
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getRightY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                false, true), 
            m_robotDrive));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
   private void configureButtonBindings() {

    // -------------------------------- CHASSIS --------------------------------------------
    
    // press right bumper button of driver joystick to stop the robot and make the wheels form an X to prevent the motion on slope
    m_driverController.rightTrigger() //added command to xbox controller for operator using command xbox code
        .whileTrue(new RunCommand(()-> m_robotDrive.setX(), m_robotDrive));

    // press left bumper for strafe true left
    m_driverController.leftBumper()
        .whileTrue(new RunCommand(()-> m_robotDrive.drive(0, 0.5, 0, false, true), m_robotDrive));

    // press right bumper for strafe true right
    m_driverController.rightBumper()
        .whileTrue(new RunCommand(()-> m_robotDrive.drive(0, -0.5, 0, false, true), m_robotDrive));
        //Instead of ".whileTrue" so the operator must hold the button, could we use one command 
        //that locks the wheels .onTrue and another command that unloacks the wheels .onTrue?

    // --------------------------------SHOOTER -----------------------------------------------
    // hold right trigger of operator joystick to shoot the note out. Release the button will stop the shooter motor 
    m_operatorController.rightTrigger()
                        .onTrue(Commands.runOnce(() ->{ m_shooter.ShooterShootNoteOut();}))
                        .onFalse(Commands.runOnce(() ->{ m_shooter.ShooterStop();}));
        
    // hold left trigger of operator joystick to feed the note in. Release the button will stop the shooter motor 
    m_operatorController.leftTrigger()
                        .onTrue(Commands.runOnce(() ->{ m_shooter.ShooterFeedNoteIn();}))
                        .onFalse(Commands.runOnce(() ->{ m_shooter.ShooterStop();}));

    // --------------------------------INCLINER -----------------------------------------------
    // incliner moves up when leftBumper is hold, stop when release

    // hold left bumper of operator joystick to turn the incliner up. Release the button will stop the incliner motor 
    m_operatorController.leftBumper()
                        .onTrue(Commands.runOnce(() ->{ m_incliner.inclinerUp();}))
                        .onFalse(Commands.runOnce(() ->{ m_incliner.inclinerStop();}));

    // hold right bumper of operator joystick to turn the incliner down. Release the button will stop the incliner motor 
    m_operatorController.rightBumper()
                        .onTrue(Commands.runOnce(() ->{ m_incliner.inclinerDown();}))
                        .onFalse(Commands.runOnce(() ->{ m_incliner.inclinerStop();}));

    // press button B of operator joystick to set the angle to intake the note (for TESTING only). Remove this if it works
    m_operatorController.b()
                        .onTrue(new RunCommand(()->{ m_incliner.setInclinerIntakeAngle();}))
                        .onFalse(Commands.runOnce(() ->{ m_incliner.inclinerStop();}));
                        

    // press button A of operator joystick to set the angle to intake the note (for TESTING only). Remove this if it works
    /*m_operatorController.a()
                        .onTrue(Commands.runOnce(() ->{ m_incliner.inclinerDown();}))
                        .onFalse(Commands.runOnce(() ->{ m_incliner.inclinerStop();}));
                        
    */
   // -------------------------------- GROUND INTAKE --------------------------------------------

    m_operatorController.povUp()
                        .onTrue(Commands.runOnce(() ->{ m_ground.GroundIntakeFeedNoteIn();}))
                        .onFalse(Commands.runOnce(() ->{ m_ground.GroundIntakeStop();}));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand(){

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
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    
  }
  
    
}
