package frc.robot.util.sim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ArmElevatorSim extends SubsystemBase {

    private static final double kElevatorKp = 5.0;
    private static final double kElevatorGearing = 10.0;
    private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    private static final double kCarriageMass = 4.0; // kg

    private static final double kMinElevatorHeight = Units.inchesToMeters(2);
    private static final double kMaxElevatorHeight = Units.inchesToMeters(46);

    private static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;

    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

    private CANSparkMax m_motor = new CANSparkMax(9, MotorType.kBrushless);

    private final ElevatorSim s_elevator = new ElevatorSim(
            m_elevatorGearbox,
            kElevatorGearing,
            kCarriageMass,
            kElevatorDrumRadius,
            kMinElevatorHeight,
            kMaxElevatorHeight,
            true);

    private final PIDController m_controller = new PIDController(kElevatorKp, 0, 0);
    private final Encoder encoder = new Encoder(0, 1);

    private final EncoderSim s_encoder = new EncoderSim(encoder);

    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
            new MechanismLigament2d(
                    "Elevator", Units.metersToInches(s_elevator.getPositionMeters()), 90));



    public ArmElevatorSim() {
        encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);

        // Elevator Sim
        SmartDashboard.putData("Elevator Sim", m_mech2d);
        SmartDashboard.putNumber("Test", 1);
        
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        s_elevator.setInput(m_motor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        s_elevator.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        s_encoder.setDistance(s_elevator.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(s_elevator.getCurrentDrawAmps()));

        // Update elevator visualization with simulated position
        m_elevatorMech2d.setLength(Units.metersToInches(s_elevator.getPositionMeters()));
    }

    @Override
    public void periodic() {

        if (RobotContainer.driverController.getAButton()) {
            // Here, we run PID control like normal, with a constant setpoint of 30in.
            double pidOutput = m_controller.calculate(encoder.getDistance(), Units.inchesToMeters(30));
            m_motor.setVoltage(pidOutput);
        } else {
            // Otherwise, we disable the motor.
            m_motor.set(0.0);
        }
    }

    public double getPosition() {
        return 0;
    }

    public double setPosition(double setpoint) {
        return 0;
    }
}
