package frc.robot.subsystems.sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.SimConstants;

public class ElevatorSimulator implements Simulatable {

    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);

    // Standard classes for controlling our elevator
    private final PIDController m_controller = new PIDController(SimConstants.ElevatorValues.kElevatorKp, 0, 0);
    private final Encoder m_encoder = new Encoder(SimConstants.ElevatorValues.kEncoderAChannel,
            SimConstants.ElevatorValues.kEncoderBChannel);
    private final PWMSparkMax m_motor = new PWMSparkMax(SimConstants.ElevatorValues.kMotorPort);

    private final Joystick m_joystick;

    // Simulation classes help us simulate what's going on, including gravity.
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            m_elevatorGearbox,
            SimConstants.ElevatorValues.kElevatorGearing,
            SimConstants.ElevatorValues.kCarriageMass,
            SimConstants.ElevatorValues.kElevatorDrumRadius,
            SimConstants.ElevatorValues.kMinElevatorHeight,
            SimConstants.ElevatorValues.kMaxElevatorHeight,
            true,
            VecBuilder.fill(0.01));
            
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

    public ElevatorSimulator(Joystick m_joystick) {
        this.m_joystick = m_joystick;
    }

    public void onInit() {
        m_encoder.setDistancePerPulse(SimConstants.ElevatorValues.kElevatorEncoderDistPerPulse);
    }

    public double getPosition() {
        return Units.metersToInches(m_elevatorSim.getPositionMeters());
    }

    public void simulate() {
        m_elevatorSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
        m_elevatorSim.update(0.020);
        m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    public void run() {
        if (m_joystick.getRawButton(2)) {
            double pidOutput = m_controller.calculate(m_encoder.getDistance(), Units.inchesToMeters(30));
            m_motor.setVoltage(pidOutput);
        } else {
            m_motor.set(0.0);
        }
    }

    public void disabledInit() {
        m_motor.set(0.0);
    }
}
