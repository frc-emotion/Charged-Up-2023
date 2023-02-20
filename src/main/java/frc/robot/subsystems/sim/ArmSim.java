package frc.robot.subsystems.sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.SimConstants;


/**
 * 2D Simulation using WPILIB's single jointed arm simulator
 * <br></br> 
 * Uses PWM SparkMax instead of CANSparkMax to simulate timesteps and voltages - (REV Physics sim is faulty)
 */

public class ArmSim {

    private final DCMotor m_armGearbox = DCMotor.getNEO(1);
    private final PIDController m_controller = new PIDController(SimConstants.ArmValues.kArmKp, 0,
            SimConstants.ArmValues.kArmKd);
    private final Encoder m_encoder = new Encoder(SimConstants.ArmValues.kEncoderAChannel,
            SimConstants.ArmValues.kEncoderBChannel);

    private final PWMSparkMax m_motor = new PWMSparkMax(SimConstants.ArmValues.kMotorPort);

    private final Joystick m_joystick;

    // Simulation classes help us simulate what's going on, including gravity.
    private static final double m_armReduction = 50;
    private static final double m_armMass = Units.lbsToKilograms(9.0); // Kilograms
    private static final double m_armLength = Units.inchesToMeters(45);

    // This arm sim represents an arm that can travel from -75 degrees (rotated down
    // front)
    // to 255 degrees (rotated down in the back).
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            m_armGearbox,
            m_armReduction,
            SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
            m_armLength,
            Units.degreesToRadians(-75),
            Units.degreesToRadians(255),
            m_armMass,
            true,
            VecBuilder.fill(SimConstants.ArmValues.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

    public ArmSim(Joystick m_joystick) {
        this.m_joystick = m_joystick;
    }

    public double getArmAngle() {
        return Units.radiansToDegrees(m_armSim.getAngleRads());
    }

    public void onInit() {
        m_encoder.setDistancePerPulse(SimConstants.ArmValues.kArmEncoderDistPerPulse);

        if (!Preferences.containsKey(SimConstants.ArmValues.kArmPositionKey)) {
            Preferences.setDouble(SimConstants.ArmValues.kArmPositionKey, SimConstants.ArmValues.armPositionDeg);
        }
        if (!Preferences.containsKey(SimConstants.ArmValues.kArmPKey)) {
            Preferences.setDouble(SimConstants.ArmValues.kArmPKey, SimConstants.ArmValues.kArmKp);
        }
    }

    public void simulate() {
        m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
        m_armSim.update(0.020);
        m_encoderSim.setDistance(m_armSim.getAngleRads());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    }
    
    public void run() {
        if (m_joystick.getRawButton(1)) {
            var pidOutput = m_controller.calculate(m_encoder.getDistance(),
                    Units.degreesToRadians(SimConstants.ArmValues.armPositionDeg));

            m_motor.setVoltage(pidOutput);
        } else {
            var pidOutput = m_controller.calculate(m_encoder.getDistance(), Units.degreesToRadians(-74));
            m_motor.setVoltage(pidOutput);
        }
    }

    public void disabledInit() {
        m_motor.set(0.0);
    }

}