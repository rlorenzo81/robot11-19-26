
package org.firstinspires.ftc.teamcode.robotsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MecanumDriveV4 {

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;

    private VoltageSensor voltageSensor;
    private Telemetry telemetry;

    // ORIGINAL V3 ACCELERATION SETTINGS
    public double forwardAcceleration = 0.30;
    public double strafeAcceleration = 0.40;
    public double turnAcceleration = 0.20;
    public double decelerationTime = 0.05;
    public double reverseAcceleration = 0.35;
    public double deadzone = 0.05;

    // ORIGINAL VOLTAGE SETTINGS
    public boolean voltageCompensation = true;
    public double moderateVoltage = 11.5;
    public double lowVoltage = 10.5;

    // NEW CURRENT-AWARE ACCELERATION SETTINGS
    public boolean currentAwareAcceleration = true;

    public double moderateDriveCurrent = 11.0;
    public double highDriveCurrent = 14.0;

    public double moderateCurrentMultiplier = 1.30;
    public double highCurrentMultiplier = 1.70;

    private double voltageMultiplier = 1.0;
    private double currentMultiplier = 1.0;

    private double batteryVoltage = 12.0;
    private double filteredVoltage = 12.0;

    private double totalDriveCurrent = 0.0;
    private double filteredDriveCurrent = 0.0;

    private double lfCurrent = 0;
    private double lbCurrent = 0;
    private double rfCurrent = 0;
    private double rbCurrent = 0;

    private long lastVoltageCheck = 0;
    private long lastCurrentCheck = 0;
    private long lastTelemetryUpdate = 0;
    private long lastUpdateTime = 0;

    private double previousX = 0;
    private double previousY = 0;
    private double previousTurn = 0;

    // ====================================
    // INITIALIZATION
    // ====================================

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        leftFront = hwMap.get(DcMotorEx.class, "lf");
        leftBack = hwMap.get(DcMotorEx.class, "lb");
        rightFront = hwMap.get(DcMotorEx.class, "rf");
        rightBack = hwMap.get(DcMotorEx.class, "rb");

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

        voltageSensor = null;

        for (VoltageSensor sensor : hwMap.voltageSensor) {

            if (sensor.getVoltage() > 0) {
                voltageSensor = sensor;
                break;
            }
        }

        if (voltageSensor != null) {
            batteryVoltage = voltageSensor.getVoltage();
            filteredVoltage = batteryVoltage;
        }

        lastVoltageCheck = 0;
        lastCurrentCheck = 0;
        lastTelemetryUpdate = 0;

        voltageMultiplier = 1.0;
        currentMultiplier = 1.0;
        filteredDriveCurrent = 0;

        stop();

        if (telemetry != null) {
            telemetry.addLine("MecanumDriveV4 Initialized");
            telemetry.update();
        }
    }

    public void init(HardwareMap hwMap) {
        init(hwMap, null);
    }

    // ====================================
    // VOLTAGE MONITORING
    // ====================================

    private void updateVoltage() {

        long now = System.nanoTime();

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

        filteredVoltage =
                filteredVoltage * 0.8 + reading * 0.2;

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

    // ====================================
    // CURRENT MONITORING
    // ====================================

    private void updateDriveCurrent() {

        long now = System.nanoTime();

        // Sample current every 50 milliseconds
        if (now - lastCurrentCheck < 50_000_000L) {
            return;
        }

        lastCurrentCheck = now;

        lfCurrent = leftFront.getCurrent(CurrentUnit.AMPS);
        lbCurrent = leftBack.getCurrent(CurrentUnit.AMPS);
        rfCurrent = rightFront.getCurrent(CurrentUnit.AMPS);
        rbCurrent = rightBack.getCurrent(CurrentUnit.AMPS);

        totalDriveCurrent =
                lfCurrent + lbCurrent
                + rfCurrent + rbCurrent;

        if (!Double.isFinite(totalDriveCurrent)
                || totalDriveCurrent < 0) {
            return;
        }

        // Respond quickly when current rises,
        // more slowly when it decreases
        double alpha =
                totalDriveCurrent > filteredDriveCurrent
                        ? 0.70
                        : 0.15;

        filteredDriveCurrent +=
                alpha * (totalDriveCurrent - filteredDriveCurrent);

        // Adjust acceleration based on current
        if (!currentAwareAcceleration) {
            currentMultiplier = 1.0;
        }
        else if (filteredDriveCurrent >= highDriveCurrent) {
            currentMultiplier = highCurrentMultiplier;
        }
        else if (filteredDriveCurrent >= moderateDriveCurrent) {
            currentMultiplier = moderateCurrentMultiplier;
        }
        else {
            currentMultiplier = 1.0;
        }
    }

    // ====================================
    // ACCELERATION LIMITER
    // ====================================

    private double rampAxis(
            double target,
            double previous,
            double accelerationTime,
            double dt) {

        // Immediate release of joystick axis
        if (Math.abs(target) < deadzone) {
            return 0;
        }

        double rampTime;

        // Controlled direction reversal
        if (previous * target < 0) {

            rampTime = reverseAcceleration
                    * voltageMultiplier
                    * currentMultiplier;

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

        // Fast deceleration
        if (Math.abs(target) < Math.abs(previous)) {

            rampTime = decelerationTime;
        }
        else {

            // NEW: current-aware acceleration
            rampTime = accelerationTime
                    * voltageMultiplier
                    * currentMultiplier;
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

    // ====================================
    // MECANUM DRIVE
    // ====================================

    public void moveBot(double x, double y, double turn) {

        updateVoltage();
        updateDriveCurrent();

        long now = System.nanoTime();

        // Immediate stopping at neutral
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

        // Apply acceleration limits
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

    // ====================================
    // CURRENT AND VOLTAGE GETTERS
    // ====================================

    public double getLFCurrent() {
        return lfCurrent;
    }

    public double getLBCurrent() {
        return lbCurrent;
    }

    public double getRFCurrent() {
        return rfCurrent;
    }

    public double getRBCurrent() {
        return rbCurrent;
    }

    public double getTotalDriveCurrent() {
        return totalDriveCurrent;
    }

    public double getFilteredDriveCurrent() {
        return filteredDriveCurrent;
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

    public double getCurrentMultiplier() {
        return currentMultiplier;
    }

    public double getCombinedAccelerationMultiplier() {
        return voltageMultiplier * currentMultiplier;
    }

    // ====================================
    // AUTOMATIC TELEMETRY
    // ====================================

    private void updateTelemetry() {

        if (telemetry == null) {
            return;
        }

        long now = System.nanoTime();

        if (now - lastTelemetryUpdate < 200_000_000L) {
            return;
        }

        lastTelemetryUpdate = now;

        telemetry.addLine("=== DRIVETRAIN MONITOR V4 ===");

        telemetry.addData(
                "Battery Voltage",
                "%.2f V",
                batteryVoltage
        );

        telemetry.addData(
                "Filtered Voltage",
                "%.2f V",
                filteredVoltage
        );

        telemetry.addData(
                "LF Current",
                "%.2f A",
                lfCurrent
        );

        telemetry.addData(
                "LB Current",
                "%.2f A",
                lbCurrent
        );

        telemetry.addData(
                "RF Current",
                "%.2f A",
                rfCurrent
        );

        telemetry.addData(
                "RB Current",
                "%.2f A",
                rbCurrent
        );

        telemetry.addData(
                "TOTAL Drive Current",
                "%.2f A",
                totalDriveCurrent
        );

        telemetry.addData(
                "Filtered Drive Current",
                "%.2f A",
                filteredDriveCurrent
        );

        telemetry.addData(
                "Voltage Multiplier",
                "%.2fx",
                voltageMultiplier
        );

        telemetry.addData(
                "Current Multiplier",
                "%.2fx",
                currentMultiplier
        );

        telemetry.addData(
                "Combined Accel Multiplier",
                "%.2fx",
                getCombinedAccelerationMultiplier()
        );

        if (batteryVoltage < 10.5) {
            telemetry.addLine("WARNING: LOW BATTERY VOLTAGE");
        }

        telemetry.update();
    }

    // ====================================
    // IMMEDIATE STOP
    // ====================================

    public void stop() {

        if (leftFront != null) {
            leftFront.setPower(0);
        }

        if (leftBack != null) {
            leftBack.setPower(0);
        }

        if (rightFront != null) {
            rightFront.setPower(0);
        }

        if (rightBack != null) {
            rightBack.setPower(0);
        }

        previousX = 0;
        previousY = 0;
        previousTurn = 0;

        lastUpdateTime = 0;
    }
}
