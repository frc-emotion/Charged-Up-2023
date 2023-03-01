package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import com.playingwithfusion.TimeOfFlight;

public class Elevator {
    private ElevatorFeedforward feedForward;
    private SparkMaxPIDController elevatorController;
    private CANSparkMax elevatorMotor;
    private double low, middle, high;
    private TrapezoidProfile profile;
    private TimeOfFlight positionSensor;
    double setpoint;

    public Elevator(){
        feedForward = new ElevatorFeedforward(5, 5, 1);
        elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
        elevatorController = elevatorMotor.getPIDController();
        positionSensor = new TimeOfFlight(Constants.ElevatorConstants.CANID);

        low = Constants.ElevatorConstants.LOWLEVEL;
        middle = Constants.ElevatorConstants.MIDDLELEVEL;
        high = Constants.ElevatorConstants.HIGHLEVEL;
    }

    //the xbox controller buttons are temporary
    //PID for each level on the elevator to ease into position
    public void setHeight(){
        if(RobotContainer.operatorController.getAButtonPressed()){
            setpoint = high;
            elevatorController.setReference(setpoint, CANSparkMax.ControlType.kPosition , 5, feedForward.calculate(10) );        
        }

        if(RobotContainer.operatorController.getBButtonPressed()){
            setpoint = middle;
            elevatorController.setReference(setpoint, CANSparkMax.ControlType.kPosition , 5, feedForward.calculate(10) );        
        }
        if(RobotContainer.operatorController.getXButtonPressed()){
            setpoint = low;
            elevatorController.setReference(setpoint, CANSparkMax.ControlType.kPosition , 5, feedForward.calculate(10) ); 
        }
    }
    //allows for manual control as backup
    public void moveElevator(){
        if(RobotContainer.operatorController.getLeftY() > 0){
            elevatorMotor.set(Constants.ElevatorConstants.ELEVATORMOTORSPEED);
        }   else if(RobotContainer.operatorController.getLeftY() < 0){
            elevatorMotor.set(-Constants.ElevatorConstants.ELEVATORMOTORSPEED);
        }
    }
    //return the height the position control calculates
    public double getHeight(){
        return positionSensor.getRange();
    }
    //converts inches to meters
    public void convertToMeters(){
        elevatorMotor.getEncoder().setPositionConversionFactor(Constants.ElevatorConstants.FACTOR);
        elevatorMotor.getEncoder().setVelocityConversionFactor(Constants.ElevatorConstants.FACTOR);
    }

    
}   

