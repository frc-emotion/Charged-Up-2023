package frc.robot;

import edu.wpi.first.math.util.Units;

public class SimConstants {

    public static final class IntakeValues {
        public static final int kMotorPort = 0;
        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;
        public static final double kIntakeReduction = 0;
        public static final double kIntakeMass = 0;
        public static final double kIntakeLength = 0;

        public static final double kIntakeUprightAngle = 70.0;
        public static final double kIntakeDownAngle = 170.0;
    }

    public static final class ElevatorValues{
        public static final int kMotorPort = 0;
        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;
        public static final double kElevatorKp = 8.0;
        public static final double kElevatorGearing = 10.0;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        public static final double kCarriageMass = 4.0; // kg
        public static final double kMinElevatorHeight = Units.inchesToMeters(2);
        public static final double kMaxElevatorHeight = Units.inchesToMeters(50);
    
        // distance per pulse = (distance per revolution) / (pulses per revolution)
        // = (Pi * D) / ppr
        public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;
    }
    
    public static final class ArmValues {
        public static final int kMotorPort = 1;
        public static final int kEncoderAChannel = 2;
        public static final int kEncoderBChannel = 3;
        public static final int kJoystickPort = 0;
    
        // The P gain for the PID controller that drives this arm.
        public static double kArmKp = 100;
        public static double kArmKd = 8;
        public static double kArmKi = 0;
      
        public static double armPositionDeg = 180.0;
      
        // distance per pulse = (angle per revolution) / (pulses per revolution)
        //  = (2 * PI rads) / (4096 pulses)
        public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
        public static final double kArmDistPerRot = 2 * Math.PI;
      

    }
}
