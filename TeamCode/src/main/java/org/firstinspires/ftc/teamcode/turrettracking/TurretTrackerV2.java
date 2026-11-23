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

public class TurretTrackerV2 {

    // ==========================================
    // HARDWARE AND PIPELINE SETTINGS
    // ==========================================

    private DcMotorEx turret;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    // These names match your Robot Configuration.
    private static final String TURRET_NAME = "turret";
    private static final String CAMERA_NAME = "limelight";
    private static final String PINPOINT_NAME = "Pinpoint";

    // Your two existing Limelight pipelines.
    public static final int RED_PIPELINE = 1;
    public static final int BLUE_PIPELINE = 2;

    private int pipelineNumber = RED_PIPELINE;

    // True = audience CELL; false = far CELL.
    private boolean audienceSide = true;

    // ==========================================
    // CLUSTER OFFSETS IN METERS
    // ==========================================

    // Array index is AprilTag ID minus 30.
    // Your measured offsets for 34-37 are saved.
    // Other clusters still need calibration.
    //
    // These are approximate camera-X corrections,
    // not full 3D rotations of the tag geometry.

    private static final double[] TAG_TO_AIM_X = {
            0.165,    // 30 Red far
            0.070,    // 31
            -0.070,    // 32
            0.165,    // 33

            0.165,  // 34 Red audience
            0.070,  // 35
            -0.070,  // 36
            -0.165,  // 37

            0.165,    // 38 Blue audience
            0.070,    // 39
            -0.070,    // 40
            -0.165,    // 41

            0.165,    // 42 Blue far
            0.070,    // 43
            -0.070,    // 44
            -0.165     // 45
    };

    // ==========================================
    // TRACKING GAINS
    // ==========================================

    // Your previously tuned values.
    private double kP = 0.04;
    private double kD = 0.000013;
    private double kFF = -0.003;

    // Separate gain for recovering a lost target.
    // Start more gently than normal tracking.
    private double recoveryKP = 0.04;

    // Desired camera aiming offset, degrees.
    private double goalX = 0.0;

    // Stop fine correction near the target.
    private double angleTolerance = 0.5;

    // Normal maximum turret power.
    private static final double MAX_POWER = 0.60;

    // Lower maximum during target recovery.
    private static final double RECOVERY_MAX_POWER = 0.30;

    // Your previous motor direction was correct.
    private static final double MOTOR_SIGN = 1.0;

    // ==========================================
    // TURRET ENCODER LIMITS
    // ==========================================

    // You measured:
    // Left = +385 ticks
    // Right = -385 ticks
    //
    // Use smaller limits for a safety margin.
    // Always initialize facing straight forward.

    private static final int LEFT_LIMIT = 350;
    private static final int RIGHT_LIMIT = -350;

    // Your measured encoder scale:
    // Approximately 385 ticks per 90 degrees.
    private static final double TICKS_PER_DEGREE =
            385.0 / 90.0;

    private int turretStartTicks = 0;
    private boolean atEncoderLimit = false;

    // ==========================================
    // TARGET RECOVERY SETTINGS
    // ==========================================

    // Maximum recovery duration without tags.
    private static final double RECOVERY_TIMEOUT = 0.75;

    // Stop using old Limelight measurements.
    private static final long MAX_RESULT_AGE_MS = 150;

    // Maximum recovery angle error considered.
    // The hard encoder limits still apply.
    private static final double MAX_RECOVERY_ERROR = 90.0;

    // ==========================================
    // TRACKING STATE
    // ==========================================

    public enum TrackingState {
        TRACKING,
        RECOVERING,
        LOST,
        LIMIT
    }

    private TrackingState state = TrackingState.LOST;

    private double error = 0.0;
    private double lastError = 0.0;
    private double power = 0.0;

    private double yawRate = 0.0;
    private double robotHeading = 0.0;
    private double ffPower = 0.0;

    private double clusterBearing = 0.0;
    private int visibleTagCount = 0;
    private boolean targetVisible = false;

    private boolean hadTarget = false;
    private boolean targetMemoryValid = false;

    // Last observed HIVE direction in world heading.
    private double rememberedTargetHeading = 0.0;

    // Predicted turret position during recovery.
    private double recoveryTargetAngle = 0.0;

    private String clusterName = "None";

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime lossTimer = new ElapsedTime();

    // ==========================================
    // INITIALIZATION
    // ==========================================

    public void init(HardwareMap hwMap, int pipeline) {

        // Get turret motor.
        turret = hwMap.get(
                DcMotorEx.class, TURRET_NAME);

        // Read encoders but control motor power directly.
        turret.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake the turret when commanded to stop.
        turret.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setPower(0);

        // IMPORTANT: turret must face straight forward.
        // This records the current position as zero.
        turretStartTicks = turret.getCurrentPosition();

        // Get existing Pinpoint.
        pinpoint = hwMap.get(
                GoBildaPinpointDriver.class, PINPOINT_NAME);

        // Do not reset your Pinpoint calibration.
        pinpoint.update();

        // Get the turret-mounted Limelight.
        limelight = hwMap.get(
                Limelight3A.class, CAMERA_NAME);

        pipelineNumber = pipeline;
        limelight.pipelineSwitch(pipelineNumber);
        limelight.start();

        resetTrackingState();
    }

    // ==========================================
    // ALLIANCE AND CELL SELECTION
    // ==========================================

    public void setPipeline(int pipeline) {

        // Only accept your configured pipelines.
        if (pipeline != RED_PIPELINE &&
                pipeline != BLUE_PIPELINE) {
            return;
        }

        if (pipeline == pipelineNumber) {
            return;
        }

        // Stop and forget the old target.
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

            // Old target memory belongs to another CELL.
            turret.setPower(0);
            resetTrackingState();
        }
    }

    public boolean isAudienceSide() {
        return audienceSide;
    }

    // First ID of the selected four-tag cluster.
    private int getFirstTagId() {

        if (pipelineNumber == RED_PIPELINE) {
            return audienceSide ? 34 : 30;
        }

        return audienceSide ? 38 : 42;
    }

    private void updateClusterName() {

        String alliance = pipelineNumber == RED_PIPELINE
                ? "Red" : "Blue";

        clusterName = alliance +
                (audienceSide ? " Audience" : " Far");
    }

    // ==========================================
    // TUNING SETTINGS
    // ==========================================

    public void setkP(double value) {
        kP = Math.max(0.0, value);
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double value) {
        kD = Math.max(0.0, value);
    }

    public double getkD() {
        return kD;
    }

    // Negative values are allowed for feedforward.
    public void setkFF(double value) {
        kFF = Range.clip(value, -0.02, 0.02);
    }

    public double getkFF() {
        return kFF;
    }

    public void setRecoveryKP(double value) {
        recoveryKP = Range.clip(value, 0.0, 0.05);
    }

    public double getRecoveryKP() {
        return recoveryKP;
    }

    public void setGoalX(double value) {
        goalX = value;
    }

    // ==========================================
    // TELEMETRY GETTERS
    // ==========================================

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    public double getYawRate() {
        return yawRate;
    }

    public double getRobotHeading() {
        return robotHeading;
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
        return state == TrackingState.RECOVERING;
    }

    public boolean isAtEncoderLimit() {
        return atEncoderLimit;
    }

    public String getClusterName() {
        return clusterName;
    }

    public TrackingState getTrackingState() {
        return state;
    }

    public double getRecoveryTargetAngle() {
        return recoveryTargetAngle;
    }

    // Encoder ticks relative to startup center.
    public int getTurretTicks() {
        return turret.getCurrentPosition()
                - turretStartTicks;
    }

    // Positive angle points left.
    private double getTurretAngle() {
        return getTurretTicks() / TICKS_PER_DEGREE;
    }

    public double getTurretAngleDegrees() {
        return getTurretAngle();
    }

    // Reset timing when tracking starts.
    public void resetTimer() {
        timer.reset();
        hadTarget = false;
        lastError = 0.0;
    }

    // ==========================================
    // MAIN TRACKING LOOP
    // ==========================================

    // Call once per OpMode loop.
    public void update() {

        // Read Pinpoint once each loop.
        pinpoint.update();

        // Robot heading, degrees.
        // Unnormalized heading supports multiple turns.
        robotHeading = pinpoint.getHeading(
                UnnormalizedAngleUnit.DEGREES);

        // Robot rotation speed, degrees/second.
        yawRate = pinpoint.getHeadingVelocity(
                UnnormalizedAngleUnit.DEGREES);

        // Reject invalid sensor data.
        boolean headingValid =
                Double.isFinite(robotHeading);

        if (!Double.isFinite(yawRate)) {
            yawRate = 0.0;
        }

        // The controller should not predict
        // anything with an invalid robot heading.
        if (!headingValid) {
            robotHeading = 0.0;
            resetTargetMemory();
            applyTurretPower(0.0, MAX_POWER);
            return;
        }

        // Existing yaw-rate compensation.
        ffPower = kFF * yawRate;

        double dt = timer.seconds();
        timer.reset();

        // Read current camera result.
        LLResult result = limelight.getLatestResult();

        // Ignore wrong-pipeline and stale results.
        boolean validResult =
                result != null
                        && result.isValid()
                        && result.getPipelineIndex() == pipelineNumber
                        && result.getStaleness() >= 0
                        && result.getStaleness() <= MAX_RESULT_AGE_MS;

        double totalX = 0.0;
        double totalZ = 0.0;
        int count = 0;

        // ======================================
        // FIND THE SELECTED APRILTAG CLUSTER
        // ======================================

        if (validResult) {

            int firstId = getFirstTagId();
            int lastId = firstId + 3;

            for (LLResultTypes.FiducialResult tag :
                    result.getFiducialResults()) {

                int id = tag.getFiducialId();

                // Ignore tags outside selected CELL.
                if (id < firstId || id > lastId) {
                    continue;
                }

                // Get camera-relative 3D pose.
                Pose3D pose =
                        tag.getTargetPoseCameraSpace();

                if (pose == null ||
                        pose.getPosition() == null) {
                    continue;
                }

                double tagX = pose.getPosition().x;
                double tagZ = pose.getPosition().z;

                if (!Double.isFinite(tagX)
                        || !Double.isFinite(tagZ)
                        || tagZ <= 0.0) {
                    continue;
                }

                // Offset from tag toward CELL center.
                double offsetX = TAG_TO_AIM_X[id - 30];

                // Combine estimated CELL positions.
                totalX += tagX + offsetX;
                totalZ += tagZ;
                count++;
            }
        }

        // ======================================
        // NORMAL TRACKING: TARGET VISIBLE
        // ======================================

        if (count > 0) {

            double averageX = totalX / count;
            double averageZ = totalZ / count;

            // Horizontal angle to CELL center.
            clusterBearing = Math.toDegrees(
                    Math.atan2(averageX, averageZ));

            targetVisible = true;
            visibleTagCount = count;
            updateClusterName();

            // Preserve the familiar PD calculation.
            error = goalX - clusterBearing;

            // Calculate motor corrections.
            double pTerm = kP * error;
            double dTerm = 0.0;

            if (hadTarget && dt > 0.001) {
                dTerm = kD *
                        (error - lastError) / dt;
            }

            double command = pTerm + dTerm + ffPower;

            // Keep yaw compensation near center.
            if (Math.abs(error) <= angleTolerance) {
                command = ffPower;
            }

            // ==================================
            // REMEMBER TARGET HEADING
            // ==================================

            // Turret angle increases to the left.
            // Positive camera bearing means right.
            //
            // World HIVE heading =
            // robot heading + turret heading
            // - corrected camera bearing.
            //
            // This assumes the camera points
            // forward relative to turret zero.

            rememberedTargetHeading =
                    robotHeading
                            + getTurretAngle()
                            - (clusterBearing - goalX);

            targetMemoryValid = true;
            lossTimer.reset();

            lastError = error;
            hadTarget = true;

            // Camera tracking is in control.
            state = TrackingState.TRACKING;

            applyTurretPower(command, MAX_POWER);
            return;
        }

        // ======================================
        // TARGET LOST: RECOVERY
        // ======================================

        targetVisible = false;
        visibleTagCount = 0;
        hadTarget = false;

        // We can only recover if the camera
        // previously observed a valid target.
        if (targetMemoryValid
                && lossTimer.seconds() < RECOVERY_TIMEOUT) {

            // Last target direction relative
            // to the current robot heading.
            recoveryTargetAngle =
                    rememberedTargetHeading - robotHeading;

            // Select the equivalent angle nearest
            // the current turret direction.
            recoveryTargetAngle =
                    getTurretAngle() +
                            wrapDegrees(
                                    recoveryTargetAngle - getTurretAngle());

            // Limit the requested pointing angle
            // to the turret's safe working range.
            double minAngle =
                    RIGHT_LIMIT / TICKS_PER_DEGREE;

            double maxAngle =
                    LEFT_LIMIT / TICKS_PER_DEGREE;

            double safeTargetAngle = Range.clip(
                    recoveryTargetAngle,
                    minAngle,
                    maxAngle);

            // Directional error to remembered target.
            double recoveryError =
                    safeTargetAngle - getTurretAngle();

            // Protect against an implausible jump.
            recoveryError = Range.clip(
                    recoveryError,
                    -MAX_RECOVERY_ERROR,
                    MAX_RECOVERY_ERROR);

            error = recoveryError;

            // Position correction plus yaw feedforward.
            double recoveryCommand =
                    recoveryKP * recoveryError + ffPower;

            // Stop P correction near the predicted angle.
            if (Math.abs(recoveryError) <= angleTolerance) {
                recoveryCommand = ffPower;
            }

            state = TrackingState.RECOVERING;

            // Use a lower recovery power limit.
            applyTurretPower(
                    recoveryCommand,
                    RECOVERY_MAX_POWER);

            return;
        }

        // ======================================
        // NO USABLE TARGET MEMORY
        // ======================================

        state = TrackingState.LOST;
        error = 0.0;
        targetMemoryValid = false;

        // No uncontrolled automatic sweep.
        applyTurretPower(0.0, MAX_POWER);
    }

    // ==========================================
    // ANGLE UTILITY
    // ==========================================

    // Convert differences to -180 to +180 deg.
    private double wrapDegrees(double angle) {

        angle = (angle + 180.0) % 360.0;

        if (angle < 0.0) {
            angle += 360.0;
        }

        return angle - 180.0;
    }

    // ==========================================
    // MOTOR POWER AND SAFETY LIMITS
    // ==========================================

    private void applyTurretPower(
            double command,
            double allowedMaxPower) {

        power = Range.clip(
                command,
                -allowedMaxPower,
                allowedMaxPower);

        // MOTOR_SIGN is retained for compatibility.
        double motorPower = MOTOR_SIGN * power;

        int ticks = getTurretTicks();
        atEncoderLimit = false;

        // Your observed encoder direction:
        // Left is positive, right is negative.
        //
        // This check assumes positive motor power
        // increases encoder ticks, as indicated
        // by your existing working controller.

        if ((ticks >= LEFT_LIMIT && motorPower > 0)
                || (ticks <= RIGHT_LIMIT && motorPower < 0)) {

            motorPower = 0.0;
            power = 0.0;
            atEncoderLimit = true;
        }

        // Motor sign was verified in your earlier tests.
        turret.setPower(motorPower);

        if (atEncoderLimit) {
            state = TrackingState.LIMIT;
        }
    }

    // ==========================================
    // RESET AND STOP
    // ==========================================

    private void resetTargetMemory() {
        targetMemoryValid = false;
        hadTarget = false;
        state = TrackingState.LOST;
    }

    private void resetTrackingState() {

        targetVisible = false;
        visibleTagCount = 0;
        clusterBearing = 0.0;

        error = 0.0;
        lastError = 0.0;
        power = 0.0;

        yawRate = 0.0;
        ffPower = 0.0;
        recoveryTargetAngle = 0.0;

        targetMemoryValid = false;
        hadTarget = false;
        atEncoderLimit = false;

        state = TrackingState.LOST;

        timer.reset();
        lossTimer.reset();

        updateClusterName();
    }

    public void stop() {

        // Stop the motor.
        if (turret != null) {
            turret.setPower(0);
        }

        // Stop the camera.
        if (limelight != null) {
            limelight.stop();
        }
    }
}
