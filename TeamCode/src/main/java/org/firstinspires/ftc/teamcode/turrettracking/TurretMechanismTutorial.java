
package org.firstinspires.ftc.teamcode.turrettracking;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class TurretMechanismTutorial {

    // =====================================
    // HARDWARE
    // =====================================

    private DcMotorEx turret;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    // Hardware Configuration names.
    private static final String TURRET_NAME = "turret";
    private static final String LIMELIGHT_NAME = "limelight";

    // CHANGE THIS to your Pinpoint Configuration name.
    private static final String PINPOINT_NAME = "Pinpoint";

    // =====================================
    // LIMELIGHT PIPELINES
    // =====================================

    public static final int RED_PIPELINE = 1;
    public static final int BLUE_PIPELINE = 2;

    private int pipelineNumber = RED_PIPELINE;

    // True = audience CELL, false = far CELL.
    private boolean audienceSide = true;

    // =====================================
    // CLUSTER OFFSETS IN METERS
    // =====================================

    // Offset from each tag toward CELL center.
    // Array index = AprilTag ID - 30.
    //
    // IDs 34-37 use your measured offsets.
    // Other groups still need calibration.
    //
    // These are simple camera-X offsets,
    // not full rotated 3D geometry.

    private static final double[] TAG_TO_AIM_X = {

            0.0,    // 30 Red far
            0.0,    // 31
            0.0,    // 32
            0.0,    // 33

            0.165,  // 34 Red audience
            0.070,  // 35
            -0.070,  // 36
            -0.165,  // 37

            0.0,    // 38 Blue audience
            0.0,    // 39
            0.0,    // 40
            0.0,    // 41

            0.0,    // 42 Blue far
            0.0,    // 43
            0.0,    // 44
            0.0     // 45
    };

    // =====================================
    // CONTROLLER SETTINGS
    // =====================================

    // Previously tested proportional gain.
    private double kP = 0.04;

    // Derivative gain remains disabled.
    private double kD = 0.0;

    // Pinpoint yaw-rate feedforward.
    // Signed value; start at zero and tune.
    private double kFF = -0.003;

    // Desired camera angle in degrees.
    private double goalX = 0.0;

    // Acceptable aiming error in degrees.
    private double angleTolerance = 0.5;

    // Maximum motor power.
    private static final double MAX_POWER = 0.60;

    // Your previous motor direction worked.
    private static final double MOTOR_SIGN = 1.0;

    // =====================================
    // TARGET LOSS SETTINGS
    // =====================================

    // Continue yaw-rate compensation briefly
    // when a previously visible target disappears.
    private static final double LOSS_GRACE_SECONDS = 0.15;

    // Reject old Limelight results.
    private static final long MAX_RESULT_AGE_MS = 150;

    // =====================================
    // TURRET TRAVEL LIMITS
    // =====================================

    // IMPORTANT:
    // Limits are OFF until calibrated.
    // Do not enable them with zero limits.
    private static final boolean ENABLE_ENCODER_LIMITS = false;

    // Encoder limits relative to startup.
    // Replace with your measured safe values.
    private static final int MIN_TURRET_TICKS = 0;
    private static final int MAX_TURRET_TICKS = 0;

    private int turretStartTicks = 0;
    private boolean atEncoderLimit = false;

    // =====================================
    // CONTROLLER STATE
    // =====================================

    private double error = 0.0;
    private double lastError = 0.0;
    private double power = 0.0;

    private double yawRate = 0.0;
    private double ffPower = 0.0;

    private double clusterBearing = 0.0;

    private int visibleTagCount = 0;
    private boolean targetVisible = false;

    private boolean hadTarget = false;
    private boolean everSeenTarget = false;
    private boolean usingLossGrace = false;

    private String clusterName = "None";

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime lossTimer = new ElapsedTime();

    // =====================================
    // INITIALIZATION
    // =====================================

    public void init(HardwareMap hwMap, int pipeline) {

        // Initialize turret motor.
        turret = hwMap.get(
                DcMotorEx.class, TURRET_NAME);

        // Direct motor-power control.
        turret.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake when stopped.
        turret.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setPower(0);

        // Record starting encoder position.
        turretStartTicks = turret.getCurrentPosition();

        // Initialize Pinpoint.
        pinpoint = hwMap.get(
                GoBildaPinpointDriver.class, PINPOINT_NAME);

        // Reuse your existing Pinpoint configuration.
        // No reset or odometry recalibration here.
        pinpoint.update();

        // Initialize Limelight.
        limelight = hwMap.get(
                Limelight3A.class, LIMELIGHT_NAME);

        pipelineNumber = pipeline;
        limelight.pipelineSwitch(pipelineNumber);

        // Start camera processing.
        limelight.start();

        resetTrackingState();
    }

    // =====================================
    // ALLIANCE AND CLUSTER SELECTION
    // =====================================

    public void setPipeline(int pipeline) {

        if (pipeline != RED_PIPELINE
                && pipeline != BLUE_PIPELINE) {
            return;
        }

        if (pipeline == pipelineNumber) {
            return;
        }

        turret.setPower(0);
        pipelineNumber = pipeline;

        limelight.pipelineSwitch(pipelineNumber);

        resetTrackingState();
    }

    public int getPipeline() {
        return pipelineNumber;
    }

    public void setAudienceSide(boolean audience) {

        if (audienceSide != audience) {
            audienceSide = audience;
            turret.setPower(0);
            resetTrackingState();
        }
    }

    public boolean isAudienceSide() {
        return audienceSide;
    }

    // Find the first ID in the selected cluster.
    private int getFirstTagId() {

        if (pipelineNumber == RED_PIPELINE) {
            return audienceSide ? 34 : 30;
        }

        return audienceSide ? 38 : 42;
    }

    private void updateClusterName() {

        String alliance =
                pipelineNumber == RED_PIPELINE
                        ? "Red" : "Blue";

        clusterName = alliance +
                (audienceSide ? " Audience" : " Far");
    }

    // =====================================
    // TUNING METHODS
    // =====================================

    public void setkP(double value) {
        kP = Math.max(0, value);
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double value) {
        kD = Math.max(0, value);
    }

    public double getkD() {
        return kD;
    }

    // kFF is signed so it can compensate
    // in whichever direction your motor needs.
    public void setkFF(double value) {
        kFF = Range.clip(value, -0.02, 0.02);
    }

    public double getkFF() {
        return kFF;
    }

    public void setGoalX(double value) {
        goalX = value;
    }

    // =====================================
    // TELEMETRY GETTERS
    // =====================================

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    public double getYawRate() {
        return yawRate;
    }

    public double getFFPower() {
        return ffPower;
    }

    public double getClusterBearing() {
        return clusterBearing;
    }

    public int getVisibleTagCount() {
        return visibleTagCount;
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public boolean isUsingLossGrace() {
        return usingLossGrace;
    }

    public boolean isAtEncoderLimit() {
        return atEncoderLimit;
    }

    public String getClusterName() {
        return clusterName;
    }

    public int getTurretTicks() {
        return turret.getCurrentPosition()
                - turretStartTicks;
    }

    public void resetTimer() {
        timer.reset();
        hadTarget = false;
        lastError = 0.0;
    }

    // =====================================
    // MAIN TRACKING UPDATE
    // =====================================

    // Run this once per TeleOp loop.
    public void update() {

        // Update Pinpoint once each loop.
        pinpoint.update();

        // Read robot rotation speed in deg/sec.
        yawRate = pinpoint.getHeadingVelocity(
                UnnormalizedAngleUnit.DEGREES);

        // Reject invalid IMU readings.
        if (!Double.isFinite(yawRate)) {
            yawRate = 0.0;
        }

        // Calculate the feedforward motor command.
        ffPower = kFF * yawRate;

        // Time since the previous control loop.
        double dt = timer.seconds();
        timer.reset();

        // Get latest Limelight detection.
        LLResult result = limelight.getLatestResult();

        // Check pipeline, validity and freshness.
        boolean validResult =
                result != null
                        && result.isValid()
                        && result.getPipelineIndex() == pipelineNumber
                        && result.getStaleness() >= 0
                        && result.getStaleness() <= MAX_RESULT_AGE_MS;

        // Start assuming no tags were found.
        double totalX = 0.0;
        double totalZ = 0.0;
        int count = 0;

        if (validResult) {

            // Find the IDs in our selected CELL.
            int firstId = getFirstTagId();
            int lastId = firstId + 3;

            // Examine each detected AprilTag.
            for (LLResultTypes.FiducialResult tag
                    : result.getFiducialResults()) {

                int id = tag.getFiducialId();

                // Ignore tags outside our CELL.
                if (id < firstId || id > lastId) {
                    continue;
                }

                // Read the tag's 3D camera pose.
                Pose3D pose =
                        tag.getTargetPoseCameraSpace();

                if (pose == null
                        || pose.getPosition() == null) {
                    continue;
                }

                // Camera X = right, Z = forward.
                double tagX = pose.getPosition().x;
                double tagZ = pose.getPosition().z;

                // Reject impossible measurements.
                if (!Double.isFinite(tagX)
                        || !Double.isFinite(tagZ)
                        || tagZ <= 0) {
                    continue;
                }

                // Apply your measured tag offset.
                double offsetX = TAG_TO_AIM_X[id - 30];

                // Estimate the CELL center.
                double estimatedX = tagX + offsetX;

                // Combine all available estimates.
                totalX += estimatedX;
                totalZ += tagZ;

                count++;
            }
        }

        // =====================================
        // IF CLUSTER IS VISIBLE
        // =====================================

        if (count > 0) {

            // Average all visible tag estimates.
            double averageX = totalX / count;
            double averageZ = totalZ / count;

            // Convert estimated CELL position
            // into a horizontal aiming angle.
            clusterBearing = Math.toDegrees(
                    Math.atan2(averageX, averageZ));

            // Remember current detection status.
            targetVisible = true;
            usingLossGrace = false;
            everSeenTarget = true;
            visibleTagCount = count;

            updateClusterName();

            // Reset time since last detection.
            lossTimer.reset();

            // Current horizontal tracking error.
            error = goalX - clusterBearing;

            // Proportional correction.
            double pTerm = kP * error;

            // Derivative correction.
            double dTerm = 0.0;

            if (hadTarget && dt > 0.001) {
                dTerm = kD *
                        (error - lastError) / dt;
            }

            // Add robot rotation feedforward.
            double command = pTerm + dTerm + ffPower;

            // Stop P correction near center,
            // but retain feedforward so the
            // turret can counter-rotate.
            if (Math.abs(error) <= angleTolerance) {
                command = ffPower;
            }

            // Save values for next update.
            lastError = error;
            hadTarget = true;

            // Send motor command.
            applyTurretPower(command);
            return;
        }

        // =====================================
        // IF CLUSTER IS LOST
        // =====================================

        targetVisible = false;
        visibleTagCount = 0;
        hadTarget = false;

        // Briefly counter-rotate using Pinpoint.
        // No stale proportional correction.
        if (everSeenTarget
                && lossTimer.seconds() < LOSS_GRACE_SECONDS) {

            usingLossGrace = true;

            applyTurretPower(ffPower);
            return;
        }

        // Stop after target-loss timeout.
        usingLossGrace = false;
        everSeenTarget = false;
        error = 0.0;

        applyTurretPower(0.0);
    }

    // =====================================
    // MOTOR OUTPUT AND TRAVEL LIMITS
    // =====================================

    private void applyTurretPower(double command) {

        // Limit total power, including feedforward.
        power = Range.clip(
                command, -MAX_POWER, MAX_POWER);

        // Account for motor wiring/direction.
        double motorPower = MOTOR_SIGN * power;

        atEncoderLimit = false;

        // Optional software travel limits.
        if (ENABLE_ENCODER_LIMITS) {

            int ticks = getTurretTicks();

            // Stop if commanded farther past a limit.
            // These comparisons assume increasing
            // encoder ticks correspond to positive power.
            // Verify on your actual turret.
            if ((ticks >= MAX_TURRET_TICKS
                    && motorPower > 0)
                    || (ticks <= MIN_TURRET_TICKS
                    && motorPower < 0)) {

                motorPower = 0.0;
                power = 0.0;
                atEncoderLimit = true;
            }
        }

        turret.setPower(motorPower);
    }

    // =====================================
    // RESET AND STOP
    // =====================================

    private void resetTrackingState() {

        targetVisible = false;
        usingLossGrace = false;
        everSeenTarget = false;
        visibleTagCount = 0;

        clusterBearing = 0.0;
        error = 0.0;
        lastError = 0.0;

        yawRate = 0.0;
        ffPower = 0.0;
        power = 0.0;

        hadTarget = false;
        atEncoderLimit = false;

        timer.reset();
        lossTimer.reset();
    }

    public void stop() {

        // Stop the turret.
        if (turret != null) {
            turret.setPower(0);
        }

        // Stop Limelight processing.
        if (limelight != null) {
            limelight.stop();
        }
    }
}
