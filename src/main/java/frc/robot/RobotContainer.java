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
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InclinerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

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

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort); //Added xbox controller for separate operator controls

  CommandXboxController m_cDriverController = new CommandXboxController(OIConstants.kDriveControllerPort); //Added Command xbox controller to be able to use built in trigger commands
  CommandXboxController m_cOperatorController = new CommandXboxController(OIConstants.kOperatorControllerPort); //Added Command xbox controller to be able to use built in trigger commands

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
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
    // press X button to stop the robot
    //new JoystickButton(m_driverController, Button.kX.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //        m_robotDrive));

    
    // shooter rotator motor is connected to port 7
    m_cDriverController.x() //added command to xbox controller for operator using command xbox code
        .whileTrue(new RunCommand(()-> m_robotDrive.setX(), m_robotDrive));
    

    // --------------------------------SHOOTER -----------------------------------------------
/*
    // shoot the note out when press Y button
    new JoystickButton(m_operatorController, Button.kY.value)
         .whileTrue(new RunCommand(
             () -> m_shooter.ShooterShootNoteOut(),
             m_shooter))
         .whileFalse(new RunCommand(
             () -> m_shooter.ShooterStop(),
             m_shooter));

    // feed the note in when press A button
    new JoystickButton(m_operatorController, Button.kA.value)
         .whileTrue(new RunCommand(
             () -> m_shooter.ShooterFeedNoteIn(),
             m_shooter))
         .whileFalse(new RunCommand(
             () -> m_shooter.ShooterStop(),
             m_shooter));
*/
m_cOperatorController
    .y()
    .onTrue(Commands.runOnce(() ->{ m_shooter.ShooterShootNoteOut();
                                  }))
    .onFalse(Commands.runOnce(() ->{ m_shooter.ShooterStop();
                                  }));
        
m_cOperatorController
    .a()
    .onTrue(Commands.runOnce(() ->{ m_shooter.ShooterFeedNoteIn();
                                  }))
    .onFalse(Commands.runOnce(() ->{ m_shooter.ShooterStop();
                                  }));
    // --------------------------------INCLINER -----------------------------------------------
    // incliner moves up when leftBumper is hold, stop when release
    new JoystickButton((m_operatorController), Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> m_incliner.inclinerUp(), m_incliner))
        .whileFalse(new RunCommand(() -> m_incliner.inclinerStop(), m_incliner));
        
    // incliner moves down when rightBumper is hold, stop when release
    new JoystickButton((m_operatorController), Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_incliner.inclinerDown(), m_incliner))
        .whileFalse(new RunCommand(() -> m_incliner.inclinerStop(), m_incliner));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
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
