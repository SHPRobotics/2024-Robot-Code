// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistancePID extends Command {
  private DriveSubsystem m_driveSubsystem;
  private boolean m_driveReversed;
  private double m_distanceMeters;
  private double m_error;
  private final double kP = 0.5;

  /** Creates a new DriveDistancePID. */
  public DriveDistancePID(DriveSubsystem driveSubsystem, boolean driveReversed, double distanceMeters){
    m_driveSubsystem = driveSubsystem;
    m_driveReversed = driveReversed;
    m_distanceMeters = distanceMeters;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get average robot position
    double encoderPosition = m_driveSubsystem.getAverageEncoderDistance();

    // calculation error
    m_error = m_distanceMeters - encoderPosition;

    double outputSpeed = m_error * kP;

    m_driveSubsystem.drive(outputSpeed * (m_driveReversed ? -1 : 1), 0, 0, false, true);

    System.out.println("error = "+ m_error+", outputSpeed = "+outputSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_error <= 0.05);
  }
}
