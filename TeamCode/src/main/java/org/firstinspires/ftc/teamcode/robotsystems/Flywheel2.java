package org.firstinspires.ftc.teamcode.robotsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Flywheel2 {

    private DcMotorEx shooter;

    private final double encoderCPM = 28;
    private final double gearRatio = 1;

    private double targetRPM = 0;

    // REV Hub PIDF coefficients - starting values
    private double kP = 200;
    private double kI = 0;
    private double kD = 0;
    private double kF = 13.7; //was 12 but 20 hit 3000 rpm

    public void init(HardwareMap hwMap) {

        shooter = hwMap.get(DcMotorEx.class, "shoot");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set velocity PIDF coefficients
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );

        shooter.setPower(0);
    }

    // Set desired flywheel RPM
    public void setTargetRPM(double rpm) {

        targetRPM = rpm;

        if (targetRPM > 0) {
            double targetTicksPerSecond =
                    (targetRPM * encoderCPM * gearRatio) / 60.0;

            shooter.setVelocity(targetTicksPerSecond);
        } else {
            stopMotors();
        }
    }

    // Get desired flywheel RPM
    public double getTargetRPM() {
        return targetRPM;
    }

    // Get actual flywheel RPM
    public double getRPM() {
        return (shooter.getVelocity() / encoderCPM) * 60 / gearRatio;
    }

    // Maintain flywheel RPM


    // Check whether flywheel is ready to shoot
    public boolean atSpeed() {
        return targetRPM > 0 &&
                Math.abs(targetRPM - getRPM()) < 20;
    }

    // Stop flywheel
    public void stopMotors() {
        shooter.setPower(0);
    }
}
