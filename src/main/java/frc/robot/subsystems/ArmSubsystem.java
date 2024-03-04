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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DigitalInput;

// Arm subsystem is used to change the angle of the shooter
public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_ArmMotor  = new CANSparkMax(ArmConstants.kArmMotorCANId, MotorType.kBrushless);
  private final RelativeEncoder m_ArmEncoder = m_ArmMotor.getEncoder();
  private boolean m_armDown, m_armUp;
  private DigitalInput m_armLimitSwitch = new DigitalInput(9);

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
    // monitor the Limit Switch status
    SmartDashboard.putBoolean("Limit Switch", m_armLimitSwitch.get());
    SmartDashboard.putNumber("Arm Encoder", m_ArmEncoder.getPosition());
  }

  // set a negative speed to make arm goes up
  public void armUp(){
    m_ArmMotor.set(-ArmConstants.kArmSpeed);
    m_armUp = true;
  }

  // set a positive speed to make arm goes down
  public void armDown(){

    m_ArmMotor.set(ArmConstants.kArmSpeed);
    m_armDown = true;
    /*if (!m_armLimitSwitch.get()){ 
      armStop();
      m_ArmEncoder.setPosition(0);
    }
    else{ 

      m_ArmMotor.set(ArmConstants.kArmSpeed);
      m_armDown = true;
    }*/
  }

  public void armStop(){
    m_armDown = true;
    m_armUp = true;
    m_ArmMotor.set(0.0);
  }

  // same as armUp() except it is a Command
  public Command armUpCmd(){
    return this.runOnce(()->{m_ArmMotor.set(-ArmConstants.kArmSpeed);
                              m_armUp = true;
                            });
  }

  // same as armDown() except it is a Command
  public Command armDownCmd(){
    return this.runOnce(()->{m_ArmMotor.set(ArmConstants.kArmSpeed);
                              m_armDown = true;
                            });
  }

  // same as armStop() except it is a Command
  public Command armStopCmd(){
    return this.runOnce(()->{m_ArmMotor.set(0.0);
                              m_armDown = true;
                              m_armUp = true;
                            });
  }

  public void setArmIntakeAngle(double angle){
    if (m_ArmEncoder.getPosition() > angle){
      armUp();
      if (Constants.DEBUG) System.out.print("Arm Up ");
    }
    else if (m_ArmEncoder.getPosition() < angle){
      armDown();
      if (Constants.DEBUG) System.out.print("Arm Down ");
    }
    else{
      armStop();
      if (Constants.DEBUG) System.out.print("Arm at target ");
    }

  }
  
  public boolean isArmUp(){
    return m_armUp;
  }
  
  public boolean isArmDown(){
    return m_armDown;
  }

  public double getArmPosition(){
    return m_ArmEncoder.getPosition();
  }

  public void setArmPositionZero(){
    m_ArmEncoder.setPosition(0);
  }

  public boolean iSLimitSwitchPressed(){
    return m_armLimitSwitch.get();
  }

  /*public void isLimitSwitch(){
    while (true) {//may not need extra while true because RobotContainer already contains .whileTrue()
      if (armLimitSwitch.get()) {
        armStop();
        m_ArmEncoder.setPosition(0);
      }
  }
  */
}
