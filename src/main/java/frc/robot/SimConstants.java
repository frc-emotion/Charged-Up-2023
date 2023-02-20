package frc.robot;

public class SimConstants {
    
    public static final class ArmValues {
        public static final int kMotorPort = 1;
        public static final int kEncoderAChannel = 2;
        public static final int kEncoderBChannel = 3;
        public static final int kJoystickPort = 0;
      
        public static final String kArmPositionKey = "ArmPosition";

        public static final String kArmPKey = "ArmP";
      
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
