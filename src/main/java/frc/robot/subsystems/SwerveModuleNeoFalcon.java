package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleNeoFalcon {

    private final WPI_TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder turningEncoder;
    private final SparkMaxPIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private double resetIteration = 0;
    private double referenceAngleRadians = 0;

    private final int ENCODER_RESET_ITERATIONS = 500;


    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.drivekS, ModuleConstants.drivekV, ModuleConstants.drivekA);

    public SwerveModuleNeoFalcon(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        
        absoluteEncoder = new CANCoder(absoluteEncoderId);
        absoluteEncoder.configMagnetOffset(Units.radiansToDegrees(absoluteEncoderOffsetRad));
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        absoluteEncoder.configSensorDirection(absoluteEncoderReversed);
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 20);
        absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100, 200);

        driveMotor = new WPI_TalonFX(driveMotorId);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setInverted(driveMotorReversed);
        driveMotor.configFactoryDefault();

        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        turningMotor.setSmartCurrentLimit(45);
        turningMotor.setSecondaryCurrentLimit(45);
        turningMotor.setInverted(turningMotorReversed);
        turningMotor.setIdleMode(IdleMode.kBrake);



        turningEncoder = turningMotor.getEncoder();
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = turningMotor.getPIDController();
        
        resetEncoders();

        turningPidController.setP(1.0);
        turningPidController.setD(0.1);
        turningPidController.setI(0.0);
        turningMotor.enableVoltageCompensation(12.0);

    }

    public double getDrivePosition() {
        return toMeter(toRot(driveMotor.getSelectedSensorPosition()));
    }

    public double getDriveVelocity() {
        return toMPS(toRPM(driveMotor.getSelectedSensorVelocity()));
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setReferenceAngle(double referenceAngleRadians){
        double currentAngleRadians = getTurningPosition();

        if (getTurningVelocity() < Math.toRadians(0.5)){
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngle = Units.degreesToRadians(absoluteEncoder.getAbsolutePosition());
                turningEncoder.setPosition(absoluteAngle);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }
        
        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        this.referenceAngleRadians = referenceAngleRadians;

        turningPidController.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);

    }

    public void setSpeed(SwerveModuleState state, boolean isOpenLoop){
        if (isOpenLoop){
            driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        } else {
          //  double velocity = MPSToFalcon(state.speedMetersPerSecond, (((ModuleConstants.kWheelDiameterMeters / 2)) * 2 * Math.PI), 1 / (ModuleConstants.kDriveMotorGearRatio));
          //  driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
        }
    }

    public double getAbsolutePostion(){
        return absoluteEncoder.getAbsolutePosition();
    }


    public void resetEncoders(){
        driveMotor.setSelectedSensorPosition(0);
        turningEncoder.setPosition(Units.degreesToRadians(getAbsolutePostion()));
    }


    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * May have to use velocity PID and a feedforward command to have more accurate driving
     * 
     * @param state
     */

    public void setDesiredState(SwerveModuleState state, boolean station){
        if (Math.abs(state.speedMetersPerSecond) < 0.3 && !station){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        //driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        setSpeed(state, true);
        //turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        setReferenceAngle(state.angle.getRadians());
        //Add Debug Here
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }


    // Unit Conversion Methods
    private double toRot(double ticks) {
        return ticks / 2048;
    }

    private double toRPM(double ticks_per_time) {
        return (ticks_per_time / 2048) * 600;
    }

    private double toMeter(double rot) {
        return rot * (ModuleConstants.kDriveEncoderRot2Meter);
    }

    private double toMPS(double rpm) {
        return rpm * (ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    }

    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }
}
