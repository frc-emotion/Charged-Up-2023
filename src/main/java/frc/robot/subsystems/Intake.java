package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax intakeMotor;

    private final ArmFeedforward feedforward;

    private static TrapezoidProfile.Constraints pivotConstraints;
    private final ProfiledPIDController pivotController;

    private static RelativeEncoder pivotEncoder;

    public Intake(){
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);

        intakeMotor.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        pivotMotor.setSmartCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setSecondaryCurrentLimit(IntakeConstants.MAX_CURRENT);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        feedforward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV); // determine feedforward constants

        pivotConstraints = new TrapezoidProfile.Constraints(IntakeConstants.MAX_INTAKE_VELOCITY, IntakeConstants.MAX_INTAKE_ACCELERATION); // CHANGE
        pivotController = new ProfiledPIDController(IntakeConstants.kPPivot, IntakeConstants.kIPivot, IntakeConstants.kDPivot, pivotConstraints); // CHANGE 

        pivotEncoder = pivotMotor.getEncoder();
    }


    public void pivot(){
        double pidVal = pivotController.calculate(pivotEncoder.getPosition());
        double ffVal = feedforward.calculate(pivotEncoder.getPosition(), IntakeConstants.MAX_INTAKE_VELOCITY); //unsure about what goes in the velocity argument
        pivotMotor.set(pidVal + ffVal);
    }

    public void pivotStop(){
        pivotMotor.set(0);
    }

    public void setDownPosition(){
        pivotController.setGoal(IntakeConstants.INTAKE_DOWN_POSITION);
    }

    public void setUpPosition(){
        pivotController.setGoal(IntakeConstants.INTAKE_UP_POSITION);
    }

    public void checkPivotStop(){
        if(pivotMotor.getOutputCurrent() > IntakeConstants.CURRENT_SPIKE_THRESHOLD){
            pivotStop();
        }
    }

    public void intakeForward(){
        intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    public void intakeReverse(){
        intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
    }

    public void intakeStop(){
        intakeMotor.set(0);
    }
    
}
    
