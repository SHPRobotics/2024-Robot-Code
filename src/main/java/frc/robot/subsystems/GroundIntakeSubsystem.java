package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;

//creates new ground subsystem
public class GroundIntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftGroundMotor, m_rightGroundMotor, m_topGroundMotor;
    //private final RelativeEncoder m_leftGroundEncoder, m_rightGroundEncoder, m_topGroundEncoder;

    public GroundIntakeSubsystem() {
        // motors
        m_leftGroundMotor = new CANSparkMax(GroundIntakeConstants.kLeftGroundIntakeMotorCANId, MotorType.kBrushless);
        m_rightGroundMotor = new CANSparkMax(GroundIntakeConstants.kRightGroundIntakeMotorCANId, MotorType.kBrushless);
        m_topGroundMotor = new CANSparkMax(GroundIntakeConstants.kTopGroundIntakeMotorCANId, MotorType.kBrushless);

         // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_leftGroundMotor.restoreFactoryDefaults();
        m_rightGroundMotor.restoreFactoryDefaults();
        m_topGroundMotor.restoreFactoryDefaults();

        // set to brake mode when motors are in idle
        m_leftGroundMotor.setIdleMode(GroundIntakeConstants.kGroundIntakeMotorIdleMode);
        m_rightGroundMotor.setIdleMode(GroundIntakeConstants.kGroundIntakeMotorIdleMode);
        m_topGroundMotor.setIdleMode(GroundIntakeConstants.kGroundIntakeMotorIdleMode);

        // set the safe current limits to motors to avoid burningkShooterMotorIdleMode
        m_leftGroundMotor.setSmartCurrentLimit(GroundIntakeConstants.kGroundIntakeMotorCurrentLimit);
        m_rightGroundMotor.setSmartCurrentLimit(GroundIntakeConstants.kGroundIntakeMotorCurrentLimit);
        m_topGroundMotor.setSmartCurrentLimit(GroundIntakeConstants.kGroundIntakeMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_leftGroundMotor.burnFlash();
        m_rightGroundMotor.burnFlash();
        m_topGroundMotor.burnFlash();

        //encoders (NO NEED FOR GROUNDINTAKE ENCODER)
        //m_leftGroundEncoder = m_leftGroundMotor.getEncoder();
        //m_rightGroundEncoder = m_rightGroundMotor.getEncoder();
        //m_topGroundEncoder = m_topGroundMotor.getEncoder();
    }

    //Displays velocity of motors
    // NO NEED TO MONITOR GROUND INTAKE SPEED SINCE IT IS ALWAYS A CONSTANT
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.putNumber("Ground_Left RPM", m_leftGroundEncoder.getVelocity());
        //SmartDashboard.putNumber("Ground_Right RPM", m_rightGroundEncoder.getVelocity());
        //SmartDashboard.putNumber("Ground_Top RPM", m_topGroundEncoder.getVelocity());
    }

    //Sets velocity for ground intake; CCW is positive for motor --- CW is a negative value for motor (-1)
    public void GroundIntakeFeedNoteIn(){
        m_leftGroundMotor.set(GroundIntakeConstants.kGroundIntakeSpeed);
        m_rightGroundMotor.set(GroundIntakeConstants.kGroundIntakeSpeed);
        m_topGroundMotor.set(-GroundIntakeConstants.kTopGroundIntakeSpeed);
    }

    public void GroundIntakeFeedNoteOut(){
        m_leftGroundMotor.set(-GroundIntakeConstants.kGroundIntakeSpeed);
        m_rightGroundMotor.set(-GroundIntakeConstants.kGroundIntakeSpeed);
        m_topGroundMotor.set(GroundIntakeConstants.kTopGroundIntakeSpeed);        
    }

    //Tells ground intake motors to stop
    public void GroundIntakeStop() {
        m_leftGroundMotor.set(0);
        m_rightGroundMotor.set(0);
        m_topGroundMotor.set(0);
    }

    // same as GroundIntakeFeedNoteIn() except it is a Command
    public Command GroundIntakeFeedNoteInCmd(){
        return this.runOnce(()->{
                                m_leftGroundMotor.set(GroundIntakeConstants.kGroundIntakeSpeed);
                                m_rightGroundMotor.set(GroundIntakeConstants.kGroundIntakeSpeed);
                                m_topGroundMotor.set(-GroundIntakeConstants.kTopGroundIntakeSpeed);
                                });
    }

    // same as GroundIntakeFeedNoteOut() except it is a Command
    public Command GroundIntakeFeedNoteOutCmd(){
        return this.runOnce(()->{
                                m_leftGroundMotor.set(-GroundIntakeConstants.kGroundIntakeSpeed);
                                m_rightGroundMotor.set(-GroundIntakeConstants.kGroundIntakeSpeed);
                                m_topGroundMotor.set(GroundIntakeConstants.kTopGroundIntakeSpeed);
                                });
    }

    // same as GroundIntakeFeedStop() except it is a Command
    public Command GroundIntakeStopCmd(){
        return this.runOnce(()->{
                                m_leftGroundMotor.set(0);
                                m_rightGroundMotor.set(0);
                                m_topGroundMotor.set(0);
                                });
    }

}