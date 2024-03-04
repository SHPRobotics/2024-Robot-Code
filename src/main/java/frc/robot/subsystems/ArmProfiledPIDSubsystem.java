// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
//import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants;

public class ArmProfiledPIDSubsystem extends ProfiledPIDSubsystem {
  // arm motor
  private final CANSparkMax m_ArmMotor  = new CANSparkMax(ArmConstants.kArmMotorCANId, MotorType.kBrushless);
  // arm encoder
  private final RelativeEncoder m_ArmEncoder = m_ArmMotor.getEncoder();
  // feeadforward
  private final ArmFeedforward m_feedforward = 
    new ArmFeedforward(ArmConstants.kSVolts, 
                        ArmConstants.kGVolts, 
                        ArmConstants.kVVoltSecondPerRad,
                        ArmConstants.kAVoltSecondSquaredPerRad);


  /** Creates a new ArmProfiledPIDSubsystem. */
  public ArmProfiledPIDSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              ArmConstants.kMaxVelocityRadPerSecond, 
              ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);

    // set Position Conversion Factor
    m_ArmEncoder.setPositionConversionFactor(ArmConstants.kEncoderDistancePerPulse);

    // start arm at rest in neutral position
    setGoal(ArmConstants.kArmOffsetRads);

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_ArmMotor.setVoltage(output + feedforward);

  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_ArmEncoder.getPosition() + ArmConstants.kArmOffsetRads;
  }
}
