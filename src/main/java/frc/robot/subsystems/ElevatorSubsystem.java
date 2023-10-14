package frc.robot.subsystems;

import java.lang.Math;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.mace.Mace;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.playingwithfusion.TimeOfFlight;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorFeedforward feedForward;
    private ProfiledPIDController elevatorController;
    private CANSparkMax elevatorMotor;
    private double low, middle, high;
    private TimeOfFlight positionSensor;
    double setpoint, pidValue, feedForwardVal;

    public ElevatorSubsystem() {
        elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.ELEVATORMOTOR_ID, MotorType.kBrushless);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(3.5, 1.5);

        elevatorController = new ProfiledPIDController(2, 0, 0, constraints);
        positionSensor = new TimeOfFlight(Constants.ElevatorConstants.CANID);
        resetHeight();

        convertToMeters();
        low = Constants.ElevatorConstants.LOWLEVEL;
        middle = Constants.ElevatorConstants.MIDDLELEVEL;
        high = Constants.ElevatorConstants.HIGHLEVEL;

        SmartDashboard.putNumber("Range from sensor", 0);
        SmartDashboard.putNumber("Position", 0);
    }

    public double getPIDOutputVolts(double setpoint) {
        elevatorController.setGoal(setpoint);
        return elevatorController.calculate(getHeight());
    }

    public void setElevatorGoal(double setpoint){
        elevatorController.setGoal(setpoint);
    }

    public double getPIDOutputVoltsAuto() {
        return elevatorController.calculate(getHeight());
    }

    public double getFeedForwardOutputVolts(double v, double a) {
        return feedForward.calculate(v, a);
    }

    // the xbox controller buttons are temporary
    // PID for each level on the elevator to ease into position
    public void setHeight() {
        if (RobotContainer.operatorController.getAButton()) {

            elevatorController.setGoal(low);
            pidValue = elevatorController.calculate(getHeight());
            // feedForwardVal = feedForward.calculate(0.02);
            // MathUtil.clamp(pidValue, 0, 12);
            elevatorMotor.set(pidValue);
        } else if (RobotContainer.operatorController.getXButton()) {
            elevatorController.setGoal(middle);
            pidValue = elevatorController.calculate(getHeight());
            // feedForwardVal = feedForward.calculate(0.02);
            // MathUtil.clamp(pidValue, 0, 12);
            elevatorMotor.set(pidValue);
        } else if (RobotContainer.operatorController.getYButton()) {
            elevatorController.setGoal(high);
            pidValue = elevatorController.calculate(getHeight());
            // feedForwardVal = feedForward.calculate(0.02);
            // MathUtil.clamp(pidValue, 0, 12);
            elevatorMotor.set(pidValue);
        } else {
            stopElevator();
        }
    }

    public void setElevatorVoltage(double volts) {
        elevatorMotor.set(volts);
    }

    // allows for manual control as backup
    public void manualMove(Supplier<Double> func) {
        {
            if (func.get() > OIConstants.kDeadband) {
                elevatorMotor.set(Constants.ElevatorConstants.ELEVATORMOTORSPEED);
            } else if (func.get() < -OIConstants.kDeadband) {
                elevatorMotor.set(-Constants.ElevatorConstants.ELEVATORMOTORSPEED);
            } else if (RobotContainer.operatorController.getAButton()) {

                elevatorController.setGoal(low);
                pidValue = elevatorController.calculate(getHeight());
                // feedForwardVal = feedForward.calculate(0.02);
                // MathUtil.clamp(pidValue, 0, 12);
                elevatorMotor.set(pidValue);
            } else if (RobotContainer.operatorController.getXButton()) {
                elevatorController.setGoal(middle);
                pidValue = elevatorController.calculate(getHeight());
                // feedForwardVal = feedForward.calculate(0.02);
                // MathUtil.clamp(pidValue, 0, 12);
                elevatorMotor.set(pidValue);
            } else if (RobotContainer.operatorController.getYButton()) {
                elevatorController.setGoal(high);
                pidValue = elevatorController.calculate(getHeight());
                // feedForwardVal = feedForward.calculate(0.02);
                // MathUtil.clamp(pidValue, 0, 12);
                elevatorMotor.set(pidValue);
            } else {
                stopElevator();
            }
        }

    }

    // return the height the position control calculates
    public double getHeight() {
        return positionSensor.getRange() * 0.001;
    }

    // converts inches to meters
    public void convertToMeters() {
        elevatorMotor.getEncoder()
                .setPositionConversionFactor(2 * Math.PI * Constants.ElevatorConstants.SPROCKETRADIUS * (1 / 8));
        elevatorMotor.getEncoder()
                .setVelocityConversionFactor(2 * Math.PI * Constants.ElevatorConstants.SPROCKETRADIUS * (1 / 8) / 60);
    }

    public void stopElevator() {
        elevatorMotor.stopMotor();
    }

    public double getError() {
        return elevatorController.getPositionError();
    }

    public void resetHeight() {
        elevatorMotor.getEncoder().setPosition(getHeight());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Range from sensor", getHeight());
    }

}