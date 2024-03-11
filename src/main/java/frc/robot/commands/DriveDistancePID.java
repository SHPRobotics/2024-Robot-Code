// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistancePID extends PIDCommand {
  /** Creates a new DriveDistancePID. */
  public DriveDistancePID(DriveSubsystem driveSubsystem, boolean driveReversed, double distanceMeters) {
    super(
        // The controller that the command will use
        new PIDController(0.215, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD),

        // This should return the measurement
        //() -> 0,
        driveSubsystem::getAverageEncoderDistance,

        // This should return the setpoint (can also be a constant)
        //() -> 0,
        distanceMeters,

        // This uses the output
        output -> {
          // Use the output here
          driveSubsystem.drive(
            output * (driveReversed ? -1 : 1),
            0,
            0,
            false,
            true);
          //SmartDashboard.putNumber("output", output);
        },
        
        // Subsystem requirements
        driveSubsystem
        );

    // Configure additional PID options by calling `getController` here.
    //getController().setTolerance(0.05);

    //driveSubsystem.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return getController().atSetpoint();
  }

}
