
package org.firstinspires.ftc.teamcode.robotsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive2 {

    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx rightFront = null;

    // Adjustable ramp times in seconds
    public double accelerationTime = 0.35;
    public double decelerationTime = 0.05;

    // Joystick deadzone for immediate stopping
    public double stopDeadzone = 0.05;

    // Previous motor powers
    private double lastLF = 0;
    private double lastLB = 0;
    private double lastRF = 0;
    private double lastRB = 0;

    private long lastUpdateNanos = 0;

    public void init(HardwareMap hwMap) {

        leftFront = hwMap.get(DcMotorEx.class, "lf");
        leftBack = hwMap.get(DcMotorEx.class, "lb");
        rightBack = hwMap.get(DcMotorEx.class, "rb");
        rightFront = hwMap.get(DcMotorEx.class, "rf");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    // Limit how quickly motor power changes
    private double rampPower(
            double target,
            double previous,
            double dt) {

        // Use faster deceleration when reducing power
        // in the same direction
        boolean slowingDown =
                Math.signum(target) == Math.signum(previous)
                && Math.abs(target) < Math.abs(previous);

        double rampTime = slowingDown
                ? decelerationTime
                : accelerationTime;

        // Reversing direction uses acceleration ramp
        if (rampTime <= 0) {
            return target;
        }

        double maxChange = dt / rampTime;

        double change = target - previous;

        change = Math.max(
                -maxChange,
                Math.min(maxChange, change)
        );

        return previous + change;
    }

    // Mecanum drive with acceleration limiting
    public void moveBot(double x, double y, double turn) {

        // Immediately stop when all controls are neutral
        if (Math.abs(x) < stopDeadzone &&
            Math.abs(y) < stopDeadzone &&
            Math.abs(turn) < stopDeadzone) {

            stop();
            return;
        }

        long now = System.nanoTime();

        double dt = (lastUpdateNanos == 0)
                ? 0.02
                : (now - lastUpdateNanos) / 1_000_000_000.0;

        lastUpdateNanos = now;

        // Prevent large jumps after loop delays
        dt = Math.max(0, Math.min(dt, 0.10));

        // Original mecanum calculations
        double denominator = Math.max(
                Math.abs(x) + Math.abs(y) + Math.abs(turn),
                1.0
        );

        double lf = (x - y + turn) / denominator;
        double lb = (-x - y + turn) / denominator;
        double rf = (-x - y - turn) / denominator;
        double rb = (x - y - turn) / denominator;

        // Apply acceleration and deceleration limits
        lastLF = rampPower(lf, lastLF, dt);
        lastLB = rampPower(lb, lastLB, dt);
        lastRF = rampPower(rf, lastRF, dt);
        lastRB = rampPower(rb, lastRB, dt);

        // Send power to motors
        leftFront.setPower(lastLF);
        leftBack.setPower(lastLB);
        rightFront.setPower(lastRF);
        rightBack.setPower(lastRB);
    }

    // Immediately stop all drive motors
    public void stop() {

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Reset ramp history
        lastLF = 0;
        lastLB = 0;
        lastRF = 0;
        lastRB = 0;

        lastUpdateNanos = 0;
    }
}
