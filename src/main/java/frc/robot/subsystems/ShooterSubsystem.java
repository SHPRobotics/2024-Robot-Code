// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//this is a test for the github!
// this is a second test for github sharing
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftMotor, m_rightMotor;
  private final RelativeEncoder m_leftEncoder, m_rightEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // motors
    m_leftMotor = new CANSparkMax(ShooterConstants.kShooterLeftMotorCANId, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(ShooterConstants.kShooterRightMotorCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    // set to brake mode when motors are in idle
    m_leftMotor.setIdleMode(ShooterConstants.kShooterMotorIdleMode);
    m_rightMotor.setIdleMode(ShooterConstants.kShooterMotorIdleMode);

    // set the safe current limits to motors to avoid burning
    m_leftMotor.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit);
    m_rightMotor.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_leftMotor.burnFlash();
    m_rightMotor.burnFlash();

    // encoders
    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_leftMotor.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter_Left RPM", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter_Right RPM", m_rightEncoder.getVelocity());

  }

  public void setSpeedHi(){
    m_leftMotor.set(ShooterConstants.kShooterHiSpeed);
    m_rightMotor.set(-ShooterConstants.kShooterHiSpeed);
  }


  public void setSpeedLow(){
    m_leftMotor.set(ShooterConstants.kShooterLowSpeed);
    m_rightMotor.set(-ShooterConstants.kShooterLowSpeed);
  }



  public Command shooterStopCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          m_leftMotor.set(0);
          m_rightMotor.set(0);
        });
  }

}
