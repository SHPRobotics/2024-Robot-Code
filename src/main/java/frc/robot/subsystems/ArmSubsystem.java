// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

// Arm subsystem is used to change the angle of the shooter
public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_ArmMotor  = new CANSparkMax(ArmConstants.kArmMotorCANId, MotorType.kBrushless);
  private final RelativeEncoder m_ArmEncoder = m_ArmMotor.getEncoder();
  private boolean m_armDown, m_armUp;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_ArmEncoder.setPosition(0);

    // set the angle limit the Arm can move (forward, reverse direction)
    m_ArmMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmConstants.kArmForwardLimit);
    m_ArmMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmConstants.kArmReverseLimit);

    // enable the softlimits (forward, reverse direction)
    m_ArmMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_ArmMotor.enableSoftLimit(SoftLimitDirection.kReverse,  false);
    
    m_ArmMotor.setIdleMode(IdleMode.kBrake);
    // set the safe current limits to motors to avoid burning
    m_ArmMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // monitor the angle of the Arm
    SmartDashboard.putNumber("Arm Encoder value", m_ArmEncoder.getPosition());
  }

  // set a positive speed to make arm goes up
  public void armUp(){
    m_ArmMotor.set(ArmConstants.kArmSpeed);
  }

  // set a negative speed to make arm goes down
  public void armDown(){
    m_ArmMotor.set(-ArmConstants.kArmSpeed);
  }

  public void setArmIntakeAngle(double angle){
    if (m_ArmEncoder.getPosition() < angle){
      armUp();
      m_armUp = true;
      if (Constants.DEBUG) System.out.print("Arm Up ");
    }
    else if (m_ArmEncoder.getPosition() > angle){
      armDown();
      m_armDown = true;
      if (Constants.DEBUG) System.out.print("Arm Down ");
    }
    else{
      armStop();
      m_armDown = true;
      m_armUp = true;
      if (Constants.DEBUG) System.out.print("Arm at target ");
    }

  }
  
  public boolean isArmUp(){
    return m_armUp;
  }
  
  public boolean isArmDown(){
    return m_armDown;
  }

  public void armStop(){
    m_ArmMotor.set(0.0);
  }

  public double getArmPosition(){
    return m_ArmEncoder.getPosition();
  }
  
}
