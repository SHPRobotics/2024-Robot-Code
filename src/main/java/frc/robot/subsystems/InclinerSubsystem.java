// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InclinerSubsystem extends SubsystemBase {
  private final CANSparkMax m_InclinerMotor = new CANSparkMax(,);
  /** Creates a new InclinerSubsystem. */
  public InclinerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}