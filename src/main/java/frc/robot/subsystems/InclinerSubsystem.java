// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InclinerConstants;


// Incliner subsystem is used to change the angle of the shooter
public class InclinerSubsystem extends SubsystemBase {
  private final CANSparkMax m_inclinerMotor  = new CANSparkMax(InclinerConstants.kInclinerMotorCANId, MotorType.kBrushless);
  private final RelativeEncoder m_inclinerEncoder = m_inclinerMotor.getEncoder();
  
  /** Creates a new InclinerSubsystem. */
  public InclinerSubsystem() {
    m_inclinerEncoder.setPosition(0);

    // set the angle limit the incliner can move (forward, reveese direction)
    m_inclinerMotor.setSoftLimit(SoftLimitDirection.kForward, (float) InclinerConstants.kInclinerForwardLimit);
    m_inclinerMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) InclinerConstants.kInclinerAngleIntake);  //kInclinerReverseLimit);

    // enable the softlimits (forward, reverse direction)
    m_inclinerMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_inclinerMotor.enableSoftLimit(SoftLimitDirection.kReverse,  false);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // monitor the angle of the incliner
    SmartDashboard.putNumber("Incliner Encoder value", m_inclinerEncoder.getPosition());
  }
/*
  public Command inclinerUp(){
    return runOnce(()->m_inclinerMotor.set(InclinerConstants.kInclinerSpeed));
  }

  public Command inclinerDown(){
    return runOnce(()->m_inclinerMotor.set(-InclinerConstants.kInclinerSpeed));
  }
*/
//Let's get started on the incliner angles
  public void inclinerUp(){
    m_inclinerMotor.set(InclinerConstants.kInclinerSpeed);
  }

  public void inclinerDown(){
    m_inclinerMotor.set(-InclinerConstants.kInclinerSpeed);
  }

  public void setInclinerIntakeAngle(){
    if (m_inclinerEncoder.getPosition()< InclinerConstants.kInclinerAngleIntake) 
      m_inclinerMotor.set(InclinerConstants.kInclinerSpeed);

    else if (m_inclinerEncoder.getPosition()> InclinerConstants.kInclinerAngleIntake)
      m_inclinerMotor.set(-InclinerConstants.kInclinerSpeed);

    else
      m_inclinerMotor.set(0);
    
  }
  
  public void inclinerStop(){
    m_inclinerMotor.set(0.0);
  }

  // set the incliner's angle to receive the note 
  public void InclinerSetAngleIntake(){
    //System.out.println("InclinerSetAngleIntake");
    //m_inclinerEncoder.setPosition(InclinerConstants.kInclinerAngleIntake);
    //m_inclinerMotor.set(0);
  }

  // set the incliner's angle to drop the note into the amplifier
  public void InclinerSetAngleAmp(){
    m_inclinerEncoder.setPosition(InclinerConstants.kInclinerAngleAmp);
  }

  // set the incliner's angle to shoot the note into the speaker
  public void InclinerSetAngleShooter(){
    m_inclinerEncoder.setPosition(InclinerConstants.kInclinerAngleShooter);
  }
}
