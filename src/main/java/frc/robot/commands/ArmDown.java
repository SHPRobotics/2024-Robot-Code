// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDown extends Command {
  ArmSubsystem m_armSubsystem;

  /** Creates a new ArmDown. */
  public ArmDown(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_armSubsystem.armDown();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_armSubsystem.iSLimitSwitchPressed()){ 
      m_armSubsystem.setArmPositionZero();
      return true;
    }
    else {
      return false;
    }
  }
}
