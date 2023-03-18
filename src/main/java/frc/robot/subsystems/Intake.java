package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax intakeMotor;

    private final ArmFeedforward feedforward;
    private final PIDController pivotController;

    private static RelativeEncoder pivotEncoder;

    public static boolean down;

    private final SlewRateLimiter rampLimiter;

    private static double INTAKE_SPEED, kPPivot, kIPivot, kDPivot;

    public Intake(){
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);

        intakeMotor.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setIdleMode(IdleMode.kCoast);
 
        pivotMotor.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = pivotMotor.getEncoder();

        feedforward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV); // determine feedforward constants
        pivotController = new PIDController(kPPivot, kIPivot, kDPivot); 
        
        down = false;

        rampLimiter = new SlewRateLimiter(-0.6, 0.6, 0.25);

        SmartDashboard.putNumber("Intake Speed", IntakeConstants.INTAKE_SPEED);
        SmartDashboard.putNumber("kP Pivot", IntakeConstants.kPPivot);
        SmartDashboard.putNumber("kI Pivot", IntakeConstants.kIPivot);
        SmartDashboard.putNumber("kD Pivot", IntakeConstants.kDPivot);
    }

    // intake pivot methods
    public void pivot(){
        double pidVal = pivotController.calculate(pivotEncoder.getPosition());
        double ffVal = feedforward.calculate(pivotEncoder.getPosition(), pivotEncoder.getVelocity());
        double volts = pidVal + ffVal;
        
        MathUtil.clamp(volts, 0, 8); // for PID testing

        pivotMotor.setVoltage(volts);
    }

    public void pivotStop(){
        pivotMotor.set(0);
    }

    public void toggleEndState(){
        down = !down;
        if(down){
            pivotController.setSetpoint(IntakeConstants.INTAKE_DOWN_POSITION);
        }
        if(!down){
            pivotController.setSetpoint(IntakeConstants.INTAKE_UP_POSITION);
        }
    }

    public void resetEncoder(){
        if(down){
            pivotEncoder.setPosition(IntakeConstants.INTAKE_DOWN_POSITION);
        }
        if(!down){
            pivotEncoder.setPosition(IntakeConstants.INTAKE_UP_POSITION);
        }
    }

    public boolean checkCurrentSpike(){
        if(pivotMotor.getOutputCurrent() > IntakeConstants.CURRENT_SPIKE_THRESHOLD){
            resetEncoder();
            return true;
        }
        else{
            return false;
        }
    }

    // intake run methods
    public void intakeForward(){
        intakeMotor.set(rampLimiter.calculate(INTAKE_SPEED));
    }

    public void intakeReverse(){
        intakeMotor.set(-rampLimiter.calculate(INTAKE_SPEED));
    }

    public void intakeStop(){
        intakeMotor.set(0);
    }

    @Override
    public void periodic(){
        INTAKE_SPEED = SmartDashboard.getNumber("Intake Speed", IntakeConstants.INTAKE_SPEED);
        kPPivot = SmartDashboard.getNumber("kP Pivot", IntakeConstants.kPPivot);
        kIPivot = SmartDashboard.getNumber("kI Pivot", IntakeConstants.kIPivot);
        kDPivot = SmartDashboard.getNumber("kD Pivot", IntakeConstants.kDPivot);

    }
}
    
