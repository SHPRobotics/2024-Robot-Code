// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistancePIDCustom2 extends Command {
  /** Creates a new DriveDistancePIDCustom2. */
  private DriveSubsystem m_driveSubsystem;
  private boolean m_driveReversed;
  private double m_distanceMeters;

  private double m_error;
  private double m_output;
  private double kP = 0.167;
  
  public DriveDistancePIDCustom2(DriveSubsystem driveSubsystem, boolean driveReversed, double distanceMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    m_error = m_distanceMeters - m_driveSubsystem.getAverageEncoderDistance();
    m_output = kP * m_error + 0.1;
    m_driveSubsystem.drive(m_output* (m_driveReversed ? -1 : 1), 0, 0, false, true);
    SmartDashboard.putNumber("Output", m_output);
    SmartDashboard.putNumber("Error", m_error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_driveSubsystem.getAverageEncoderDistance() >= (m_distanceMeters)) {
      return true;
    }
    else 
      return false;
  }
}
