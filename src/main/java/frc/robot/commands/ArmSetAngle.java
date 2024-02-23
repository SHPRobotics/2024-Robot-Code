// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetAngle extends Command {
  ArmSubsystem m_armSubsystem;
  double m_angle;
  boolean m_armUp, m_armDown;

  /** Creates a new ArmSetAngle. */
  public ArmSetAngle(ArmSubsystem armSubsystem, double angle) {
    m_armSubsystem = armSubsystem;
    m_angle = angle;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armUp = false;
    m_armDown = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_armSubsystem.getArmPosition() < m_angle){ 
      m_armSubsystem.armUp();
      m_armUp = true;
      System.out.println("Arm Up at position: "+m_armSubsystem.getArmPosition()+", target angle = "+ m_angle);
    }
    else if(m_armSubsystem.getArmPosition() > m_angle){ 
      m_armSubsystem.armDown();
      m_armDown = true;
      System.out.println("Arm Down at position: "+m_armSubsystem.getArmPosition()+", target angle = "+ m_angle);
    }
    else {
      m_armSubsystem.armStop();
      m_armDown = true;
      m_armUp = true;
      System.out.println("Arm at position: "+m_armSubsystem.getArmPosition()+", target angle = "+ m_angle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_armUp && m_armDown) return true;
    else return false;
  }
}
