
package org.firstinspires.ftc.teamcode.robotsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MecanumDrive3 {

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;

    private VoltageSensor voltageSensor;
    private Telemetry telemetry;

    // ======================================
    // ACCELERATION SETTINGS
    // ======================================

    public double forwardAcceleration = 0.30;
    public double strafeAcceleration = 0.40;
    public double turnAcceleration = 0.20;

    public double decelerationTime = 0.05;
    public double reverseAcceleration = 0.35;

    public double deadzone = 0.05;

    // Automatic voltage-based acceleration
    public boolean voltageCompensation = true;

    // ======================================
    // VOLTAGE SETTINGS
    // ======================================

    public double moderateVoltage = 11.5;
    public double lowVoltage = 10.5;

    private double voltageMultiplier = 1.0;

    private double batteryVoltage = 12.0;
    private double filteredVoltage = 12.0;

    private long lastVoltageCheck = 0;
    private long lastTelemetryUpdate = 0;

    // ======================================
    // PREVIOUS DRIVE COMMANDS
    // ======================================

    private double previousX = 0;
    private double previousY = 0;
    private double previousTurn = 0;

    private long lastUpdateTime = 0;

    // ======================================
    // INITIALIZATION
    // ======================================

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        leftFront = hwMap.get(DcMotorEx.class, "lf");
        leftBack = hwMap.get(DcMotorEx.class, "lb");
        rightFront = hwMap.get(DcMotorEx.class, "rf");
        rightBack = hwMap.get(DcMotorEx.class, "rb");

        leftFront.setDirection(
                DcMotorSimple.Direction.FORWARD);

        leftBack.setDirection(
                DcMotorSimple.Direction.FORWARD);

        rightFront.setDirection(
                DcMotorSimple.Direction.REVERSE);

        rightBack.setDirection(
                DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // Locate battery voltage sensor
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();

            if (voltage > 0) {
                voltageSensor = sensor;
                break;
            }
        }

        if (voltageSensor != null) {
            batteryVoltage = voltageSensor.getVoltage();
            filteredVoltage = batteryVoltage;
        }

        stop();

        telemetry.addLine("MecanumDrive3 Initialized");
        telemetry.update();
    }

    // Optional initialization without telemetry
    public void init(HardwareMap hwMap) {
        init(hwMap, nullSafeTelemetry());
    }

    private Telemetry nullSafeTelemetry() {
        // Telemetry must be supplied for automatic display.
        // Use init(hardwareMap, telemetry) in TeleOp.
        return null;
    }

    // ======================================
    // BATTERY VOLTAGE MONITORING
    // ======================================

    private void updateVoltage() {

        long now = System.nanoTime();

        // Check voltage every 100 ms
        if (now - lastVoltageCheck < 100_000_000L) {
            return;
        }

        lastVoltageCheck = now;

        if (voltageSensor == null) {
            voltageMultiplier = 1.0;
            return;
        }

        double reading = voltageSensor.getVoltage();

        if (reading <= 0) {
            return;
        }

        batteryVoltage = reading;

        // Smooth voltage readings
        filteredVoltage =
                filteredVoltage * 0.8
                + reading * 0.2;

        if (!voltageCompensation) {
            voltageMultiplier = 1.0;
        }
        else if (filteredVoltage < lowVoltage) {
            voltageMultiplier = 2.0;
        }
        else if (filteredVoltage < moderateVoltage) {
            voltageMultiplier = 1.5;
        }
        else {
            voltageMultiplier = 1.0;
        }
    }

    // ======================================
    // ACCELERATION LIMITER
    // ======================================

    private double rampAxis(
            double target,
            double previous,
            double accelerationTime,
            double dt) {

        if (Math.abs(target) < deadzone) {
            return 0;
        }

        double rampTime;

        // Reversing direction
        if (previous * target < 0) {

            rampTime = reverseAcceleration
                    * voltageMultiplier;

            if (rampTime <= 0) {
                return target;
            }

            double maxStep = dt / rampTime;

            if (Math.abs(previous) <= maxStep) {
                return 0;
            }

            return previous
                    - Math.signum(previous) * maxStep;
        }

        // Reduce power quickly
        if (Math.abs(target) < Math.abs(previous)) {
            rampTime = decelerationTime;
        }
        else {
            rampTime = accelerationTime
                    * voltageMultiplier;
        }

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

    // ======================================
    // MECANUM DRIVE
    // ======================================

    public void moveBot(double x, double y, double turn) {

        updateVoltage();

        long now = System.nanoTime();

        // Stop immediately when controls are neutral
        if (Math.abs(x) < deadzone &&
            Math.abs(y) < deadzone &&
            Math.abs(turn) < deadzone) {

            stop();
            updateTelemetry();
            return;
        }

        double dt;

        if (lastUpdateTime == 0) {
            dt = 0.02;
        }
        else {
            dt = (now - lastUpdateTime)
                    / 1_000_000_000.0;
        }

        lastUpdateTime = now;

        dt = Math.max(0.001, Math.min(dt, 0.10));

        // Apply separate acceleration limits
        previousX = rampAxis(
                x,
                previousX,
                strafeAcceleration,
                dt
        );

        previousY = rampAxis(
                y,
                previousY,
                forwardAcceleration,
                dt
        );

        previousTurn = rampAxis(
                turn,
                previousTurn,
                turnAcceleration,
                dt
        );

        // Original mecanum calculations
        double denominator = Math.max(
                Math.abs(previousX)
                + Math.abs(previousY)
                + Math.abs(previousTurn),
                1.0
        );

        double lf =
                (previousX - previousY + previousTurn)
                        / denominator;

        double lb =
                (-previousX - previousY + previousTurn)
                        / denominator;

        double rf =
                (-previousX - previousY - previousTurn)
                        / denominator;

        double rb =
                (previousX - previousY - previousTurn)
                        / denominator;

        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);

        updateTelemetry();
    }

    // ======================================
    // MOTOR CURRENT MONITORING
    // ======================================

    public double getLFCurrent() {
        return leftFront.getCurrent(CurrentUnit.AMPS);
    }

    public double getLBCurrent() {
        return leftBack.getCurrent(CurrentUnit.AMPS);
    }

    public double getRFCurrent() {
        return rightFront.getCurrent(CurrentUnit.AMPS);
    }

    public double getRBCurrent() {
        return rightBack.getCurrent(CurrentUnit.AMPS);
    }

    public double getTotalDriveCurrent() {
        return getLFCurrent()
                + getLBCurrent()
                + getRFCurrent()
                + getRBCurrent();
    }

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public double getFilteredVoltage() {
        return filteredVoltage;
    }

    public double getVoltageMultiplier() {
        return voltageMultiplier;
    }

    // ======================================
    // AUTOMATIC DRIVER STATION TELEMETRY
    // ======================================

    private void updateTelemetry() {

        if (telemetry == null) {
            return;
        }

        long now = System.nanoTime();

        // Update display every 200 ms
        if (now - lastTelemetryUpdate < 200_000_000L) {
            return;
        }

        lastTelemetryUpdate = now;

        double lfCurrent = getLFCurrent();
        double lbCurrent = getLBCurrent();
        double rfCurrent = getRFCurrent();
        double rbCurrent = getRBCurrent();

        double totalCurrent =
                lfCurrent + lbCurrent
                + rfCurrent + rbCurrent;

        telemetry.addLine("=== DRIVETRAIN MONITOR ===");

        telemetry.addData(
                "Battery Voltage",
                "%.2f V",
                batteryVoltage);

        telemetry.addData(
                "Filtered Voltage",
                "%.2f V",
                filteredVoltage);

        telemetry.addData(
                "LF Current",
                "%.2f A",
                lfCurrent);

        telemetry.addData(
                "LB Current",
                "%.2f A",
                lbCurrent);

        telemetry.addData(
                "RF Current",
                "%.2f A",
                rfCurrent);

        telemetry.addData(
                "RB Current",
                "%.2f A",
                rbCurrent);

        telemetry.addData(
                "TOTAL Drive Current",
                "%.2f A",
                totalCurrent);

        telemetry.addData(
                "Acceleration Multiplier",
                "%.1f",
                voltageMultiplier);

        if (batteryVoltage < 10.5) {
            telemetry.addLine(
                    "WARNING: LOW BATTERY VOLTAGE");
        }

        telemetry.update();
    }

    // ======================================
    // IMMEDIATE STOP
    // ======================================

    public void stop() {

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        previousX = 0;
        previousY = 0;
        previousTurn = 0;

        lastUpdateTime = 0;
    }
}
