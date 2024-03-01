// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 * DATE CHANGE  CHANGE BY   REASON
 * -----------  ---------   ------------------------------------------------------------------------------------------
 * 02/16/2024   TRINH2      -Add 2 Auton. commands in Autos.java 
 *                              1. driveDistanceAuto(driveSubsystem, driveReversed, distanceMeters): drive half speed Forward (if driveReversed=false)
 *                                  or Reverse (if driveReversed=true) for a distance distanceMeters in meter
 *                              2. driveAlongPathAuto(driveSubsystem) to drive the robot along a predefined path
 *                          -Add Command method getAverageEncoderDistance() in DriveSubsystem.java to get the distance the robot ran since the last encoder reset
 *                              and Command method driveAlongPathAuto() to drive robot along a predefined path
 *                          -Add a drop-down list box (chooser) in RobotContainer.java to allow the user to choose which auton. command to run
 *                              This chooser appears in Shuffleboard under 'Autonomous' tab
 * 02/22/2024   TRINH2      -Change all occurences of 'Incliner' to 'Arm'. I think 'Arm' sounds better than 'Incliner'
 *                          -Per Tony's request, left stick drives forward/backward, right stick to turn (as before)
 * -------------------------------------------------------------------------------------------------------------------
 * ACTION         BUTTON          XBOXCONTROLLER    TO DO THIS
 * ------------   -------------   --------------    ---------------------------------------
 * Press & Hold   Right Trigger   Driver            Stop robot and lock wheel to form an 'X'
 * Press & Hold   Left Bumper     Driver            Strafe left
 * Press & Hold   Right Bumper    Driver            Strafe right 
 * 
 * Press & Hold   Right Trigger   Operator          Shoot the note out
 * Press & Hold   Left Trigger    Operator          Feed the note in
 * Press & Hold   Left Bumper     Operator          Move Arm up
 * Press & Hold   Right Bumper    Operator          Move Arm down
 * Press & Hold   X               Operator          Move arm to Source position
 * Press & Hold   A               Operator          Move arm to Amp position
 * Press & Hold   Y               Operator          Move arm to Speaker position
 * Press & Hold   DPad Up         Operator          Ground intake takes note in
 * Press & Hold   DPad Down       Operator          Ground intake pushes note out
 */
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmSetAngle;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  // Arm subsystem
  private final ArmSubsystem m_arm = new ArmSubsystem();
  // Ground Intake Subsystem
  private final GroundIntakeSubsystem m_ground = new GroundIntakeSubsystem();
  // The driver's controller

  // command to drive the robot forward for 1.0 meters (Note: kAutoDriveReversed, kAutoDriveDistanceMeters are defined in Constants.java in case we want to change the direction and distance)
  private final Command m_driveDistanceAuto = Autos.driveDistanceAuto(m_robotDrive, false, 5.0);

  // command to drive the robot along a predefined path
  private final Command m_driveAlongPathAuto = Autos.driveAlongPathAuto(m_robotDrive);

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
  // Red 2
            -shoot note into speaker
            -drive reverse->(parallel)->run ground intake motor
            -drive foward
            -shoot note into speaker
  // Red 3
            -shoot note into speaker
            -drive reverse
            -turn counterclockwise
            -drive reverse->(parallel)->run ground intake motor 
            -drive foward
            -turn clockwise
            -drive foward
            -shoot note into speaker  
  // Blue 1
  // Blue 2
  // Blue 3
*/


  // a chooser (similar to a dropdown list) for user to select which autonomous command to run
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  //Limit Switches
  //DigitalInput bottomShooterLimit = new DigitalInput(0);

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
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false, true), 
            m_robotDrive));

    // add driveDistanceAuto commands to the command chooser as the default command
    m_chooser.setDefaultOption("Drive a Distance", m_driveDistanceAuto);
    // add 2nd auton command
    m_chooser.addOption("Drive Along a Path", m_driveAlongPathAuto);

    // put the chooser on the Dashboard under "Autonomous" tab. If tab not exist, it creates the tab
    Shuffleboard.getTab("Autonomous").add(m_chooser);

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
                        .onTrue(Commands.runOnce(() ->{ m_shooter.shooterShootNoteOut();}))
                        .onFalse(Commands.runOnce(() ->{ m_shooter.shooterStop();}));
        
    // hold left trigger of operator joystick to feed the note in. Release the button will stop the shooter motor 
    m_operatorController.leftTrigger()
                        .onTrue(Commands.runOnce(() ->{ m_shooter.shooterFeedNoteIn();}))
                        .onFalse(Commands.runOnce(() ->{ m_shooter.shooterStop();}));

    // --------------------------------Arm -----------------------------------------------
    // Arm moves up when leftBumper is hold, stop when release

    // hold left bumper of operator joystick to turn the Arm down. Release the button will stop the Arm motor 
    m_operatorController.leftBumper()
                        .onTrue(Commands.runOnce(() ->{ m_arm.armDown();}))
                        .onFalse(Commands.runOnce(() ->{ m_arm.armStop();}));

    // hold right bumper of operator joystick to turn the Arm up. Release the button will stop the Arm motor 
    m_operatorController.rightBumper()
                        .onTrue(Commands.runOnce(() ->{ m_arm.armUp();}))
                        .onFalse(Commands.runOnce(() ->{ m_arm.armStop();}));

    // press button X of operator joystick to set the arm to source angle
    m_operatorController.x()
                        .whileTrue(new ArmSetAngle(m_arm, ArmConstants.kArmAngleSource))
                        ;
   
////
    // press button B of operator joystick to set the arm to its neutral position
    // command ArmSetAngle will run only if LimitSwitch is not pressed
    // m_operatorController.b()
    //                    .whileTrue(new ArmSetAngle(m_arm, ArmConstants.kArmAngleNeutral)
                        //              .unless(() -> !m_arm.iSLimitSwitchPressed()))
                        //.whileTrue(new RunCommand(()-> m_arm.isLimitSwitch(), m_arm))

                        ;
                        //while (true){if (ArmSubsystem.isLimitSwitch){m_ArmEncoder.reset();}}


                        // neutral or left shoulder...
                        // continuously check if the button is pressed, and if the button is pressed, stop the motor immediately and set the encoder value to 0
                        
                      

////
    // press button A of operator joystick to set the arm to the Amp position
    m_operatorController.a()
                        .whileTrue(new ArmSetAngle(m_arm, ArmConstants.kArmAngleAmp))
                        ;

    // press button Y of operator joystick to set the arm to the Speaker position
    m_operatorController.y()
                        .whileTrue(new ArmSetAngle(m_arm, ArmConstants.kArmAngleSpeaker))
                        ;                    
                        

   // -------------------------------- GROUND INTAKE --------------------------------------------

    m_operatorController.povUp()
                        .onTrue(Commands.runOnce(() ->{ m_ground.GroundIntakeFeedNoteIn();}))
                        .onFalse(Commands.runOnce(() ->{ m_ground.GroundIntakeStop();}));
    // press DPad Down to push the note out
    m_operatorController.povDown()
                        .onTrue(Commands.runOnce(() ->{ m_ground.GroundIntakeFeedNoteOut();}))
                        .onFalse(Commands.runOnce(() ->{ m_ground.GroundIntakeStop();}));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand(){
    return m_chooser.getSelected();    
  }
  
    
}
