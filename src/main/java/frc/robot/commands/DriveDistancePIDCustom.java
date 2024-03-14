// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistancePIDCustom extends Command {
  private DriveSubsystem m_driveSubsystem;
  private boolean m_driveReversed;
  private double m_distanceMeters;

  private double m_error;
  private double m_errorSum = 0.0;
  private double m_lastTimeStamp = 0.0;
  private double m_lastError = 0.0;

  // initial guess of kP: 
  // at 0 meter, error = distanceMeters - 0, we want output = 50% = 0.5
  // motor output = kP x error
  //          0.5 = kP x distanceMeters
  // if distanceMeters = 10, kP = 0.5/10 = 0.05
  private final double kP = 0.2; //0.05 (undershoot), 20 (oscillate), 10 (small oscillation), 0.5 (error = 0.2)
  // initial guess of kI:
  // get the last error when only kP was involved, say 0.2, 
  // we want the integral term to increase 10% (0.1) every 1 sec
  // errorsum = error x dt x updateCount
  //          = 0.2 x 1/50 x 50 (code updates 50 times every sec)
  //          = 0.2
  // Integral output = kI x errorsum
  //            0.1  = kI x 0.2
  //              kI = 0.1/0.2 = 0.5
  // 0.5 (overshoot), 0.05 (less overshoot), 5 (oscillate)
  // 0.5 w/ iLimit=1 is the best
  private final double kI = 0.0;    //0.5;  
  // 0.01 (slow down error rate but not enough), 1 (oscillate), 0.1 is the best
  private final double kD = 0.0;    //0.01; 
  private final double iLimit = 1;

  /** Creates a new DriveDistancePID. */
  public DriveDistancePIDCustom(DriveSubsystem driveSubsystem, boolean driveReversed, double distanceMeters, double driveAngleDeg){
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
    m_errorSum = 0.0;
    m_lastError = 0.0;
    m_lastTimeStamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get average robot position
    double sensorPosition = m_driveSubsystem.getAverageEncoderDistance();

    // calculation error
    double m_error = m_distanceMeters - sensorPosition;
    double dt = Timer.getFPGATimestamp() - m_lastTimeStamp;

    // add error only when error is small (ie < iLimit meter)
    if (Math.abs(m_error) < iLimit){
      m_errorSum += m_error * dt;
    }

    double errorRate = (m_error - m_lastError) / dt;

    double outputSpeed = kP * m_error + kI * m_errorSum + kD * errorRate;

    m_driveSubsystem.drive(outputSpeed * (m_driveReversed ? -1 : 1),  // * Math.sin(m_driveAngleDeg),
                           0, //outputSpeed * (m_driveReversed ? -1 : 1) * Math.cos(m_driveAngleDeg),
                       0, false, true);

    SmartDashboard.putNumber("m_distanceMeters", m_distanceMeters);
    SmartDashboard.putNumber("error", m_error);
    SmartDashboard.putNumber("outputSpeed", outputSpeed);
    SmartDashboard.putNumber("sensorPosition", sensorPosition);
    System.out.println(m_error+", " + outputSpeed +", "+sensorPosition);

    // update last - variables
    m_lastTimeStamp = Timer.getFPGATimestamp();
    m_lastError = m_error;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setX();
    System.out.println("command finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_error <= 0.05);   // tolerance 5 cm
  }
}
