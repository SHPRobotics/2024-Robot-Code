package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InclinerConstants;
import frc.robot.Constants.ShooterConstants;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftGroundMotor, m_rightGroundMotor;
    private final RelativeEncoder m_leftGroundEncoder, m_rightGroundEncoder;

    public GroundIntakeSubsystem() {
        // motors
        m_leftGroundMotor = new CANSparkMax(GroundIntakeConstants.kLeftGroundIntakeCANId, MotorType.kBrushless);
        m_rightGroundMotor = new CANSparkMax();
    }

}