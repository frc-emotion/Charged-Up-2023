package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsytem;

public class SwerveXboxCommand extends CommandBase {

    private final SwerveSubsytem swerveSubsytem;
    private final Supplier<Double> xSpdFunc, ySpdFunc, turningSpdFunc, hardRight, hardLeft;
    private final Supplier<Boolean> fieldOrientedFunc, slowModeFunc, turboModeFunc;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private double[] speeds;

    public SwerveXboxCommand(SwerveSubsytem swerveSubsytem,
            Supplier<Double> xSpdFunc, Supplier<Double> ySpdFunc, Supplier<Double> turningSpdFunc,
            Supplier<Boolean> fieldOrientedFunc, Supplier<Boolean> slowModeFunc, Supplier<Boolean> turboModeFunc,
            Supplier<Double> hardLeft, Supplier<Double> hardRight

    ) {

        this.hardLeft = hardLeft;
        this.hardRight = hardRight;
        this.swerveSubsytem = swerveSubsytem;
        this.xSpdFunc = xSpdFunc;
        this.ySpdFunc = ySpdFunc;
        this.turningSpdFunc = turningSpdFunc;
        this.fieldOrientedFunc = fieldOrientedFunc;
        this.slowModeFunc = slowModeFunc;
        this.turboModeFunc = turboModeFunc;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsytem);
    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunc.get();
        double ySpeed = ySpdFunc.get();
        double turningSpeed = turningSpdFunc.get();
        double currentTranslationalSpeed, currentAngularSpeed;

        speeds = swerveSubsytem.getSpeedType();

        currentTranslationalSpeed = speeds[0];
        currentAngularSpeed = speeds[1];
        
        if (slowModeFunc.get()) {
            currentTranslationalSpeed = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 4;
            currentAngularSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 4;
        } else if (turboModeFunc.get()) {
            // If driver is farzad
            currentTranslationalSpeed = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            currentAngularSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        }
    

        // deadBand
        xSpeed = Math.abs(xSpeed) > (OIConstants.kDeadband / 2) ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > (OIConstants.kDeadband / 2) ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * currentTranslationalSpeed;
        ySpeed = yLimiter.calculate(ySpeed) * currentTranslationalSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * currentAngularSpeed;

        ChassisSpeeds robotSpeeds;

        if (fieldOrientedFunc.get()) {
            if (hardLeft.get() > 0.2) {
                robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.8, 0, swerveSubsytem.getRotation2d());
            } else if (hardRight.get() > 0.2) {
                robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, swerveSubsytem.getRotation2d());
            } else {
                robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
                        swerveSubsytem.getRotation2d());
            }
        } else {

            robotSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        swerveSubsytem.setChassisSpeeds(robotSpeeds);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotSpeeds);
        swerveSubsytem.setModuleStates(moduleStates);
    }


    @Override
    public void end(boolean interrupted) {
        swerveSubsytem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
