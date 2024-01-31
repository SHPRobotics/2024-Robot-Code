// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterPIDSubsystem extends PIDSubsystem {
  // motors
  private final CANSparkMax m_shooterLeftMotor = new CANSparkMax(ShooterConstants.kShooterLeftMotorCANId, MotorType.kBrushless);
  private final CANSparkMax m_shooterRightMotor = new CANSparkMax(ShooterConstants.kShooterRightMotorCANId, MotorType.kBrushless);
  // encoders
  private final RelativeEncoder m_shooterLeftEncoder = m_shooterLeftMotor.getEncoder();
  private final RelativeEncoder m_shooterRighttEncoder = m_shooterRightMotor.getEncoder();
  // feedforward
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondPerRotation);

  /** Creates a new ShooterPIDSubsystem. */
  public ShooterPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
    m_shooterLeftEncoder.setPositionConversionFactor(ShooterConstants.kEncoderDistancePerPulse);
    m_shooterRighttEncoder.setPositionConversionFactor(ShooterConstants.kEncoderDistancePerPulse);

    setSetpoint(ShooterConstants.kShooterTargetRPS);

  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_shooterLeftMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_shooterLeftEncoder.getVelocity();
  }

  public boolean atSetPoint(){
    return m_controller.atSetpoint();
  }

}
