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
        public static final int kMotorPort = 1;
        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;
        public static final double kElevatorKp = 8.0;
        public static final double kElevatorGearing = 10.0;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        public static final double kCarriageMass = 4.0; // kg
        public static final double kMinElevatorHeight = Units.inchesToMeters(2);
        public static final double kMaxElevatorHeight = Units.inchesToMeters(200);
    
        // distance per pulse = (distance per revolution) / (pulses per revolution)
        // = (Pi * D) / ppr
        public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;

        public static double kS = 0.5;
        public static double kG = 1.29;
        public static double kA = 3.84;
        public static double kV = 0.21;
    }
    
    public static final class ArmValues {
        public static final int kMotorPort = 2;
        public static final int kEncoderAChannel = 2;
        public static final int kEncoderBChannel = 3;
        public static final int kJoystickPort = 3;
    
        // The P gain for the PID controller that drives this arm.
        public static double kArmKp = 100;
        public static double kArmKd = 8;
        public static double kArmKi = 0;
      
        public static double armPositionDeg = 180.0;
      
        // distance per pulse = (angle per revolution) / (pulses per revolution)
        //  = (2 * PI rads) / (4096 pulses)
        public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
        public static final double kArmDistPerRot = 2 * Math.PI;

        public static double kS = 0.5;
        public static double kG = 0.98;
        public static double kA = 1.09;
        public static double kV = 0.05;
    
    }

    //1 rot = 2 * pi * r  mete

    private double metertoRot(double meter){
        return meter ; 
    }

    public static double armMPStoRPM(double meters){
        return armMeterToRot(meters) / 60;
    }

    public static double armMPSStoRadSS(double meters){
        return Units.rotationsPerMinuteToRadiansPerSecond(armMPStoRPM(meters)) / 60;
    }


    public static double armMeterToRot(double meters){
        return meters * (1 / (2 * Math.PI * Units.inchesToMeters(45))) * 56.8;
    }

    public static double armRadToMeter(double rad){
        return (1 / 56.8)  * (Units.radiansToRotations(rad)) * 2 * Math.PI * Units.inchesToMeters(45);
    }


}
