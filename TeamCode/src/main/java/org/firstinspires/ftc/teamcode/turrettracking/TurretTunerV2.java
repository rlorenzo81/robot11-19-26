
package org.firstinspires.ftc.teamcode.turrettracking;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotsystems.Pollinator;
@Disabled
@TeleOp(name = "Turret Tuner V2")
public class TurretTunerV2 extends OpMode {

    // ==========================================
    // ROBOT SYSTEMS
    // ==========================================

    // Your existing mecanum drivetrain.
    private final MecanumDrive drive =
            new MecanumDrive();

    Pollinator pollinate = new Pollinator();

    // New V2 turret mechanism.
    private final TurretTrackerV2 turret =
            new TurretTrackerV2();

    // ==========================================
    // ALLIANCE / CELL
    // ==========================================

    private boolean redAlliance = true;
    private boolean audienceSide = true;

    // ==========================================
    // TUNING STEP SIZES
    // ==========================================

    // Original proportional/derivative step sizes.
    private final double[] stepSizes = {
            0.1,
            0.01,
            0.001,
            0.0001,
            0.00001,
            0.000001,
            0.0000001
    };

    private int stepIndex = 2;

    // Step size for Pinpoint feedforward.
    private static final double FF_STEP = 0.0001;

    // Step size for recovery proportional gain.
    private static final double RECOVERY_STEP = 0.001;

    // Avoid repeated trigger adjustments
    // while a trigger remains held.
    private boolean previousLeftTrigger = false;
    private boolean previousRightTrigger = false;

    // ==========================================
    // INITIALIZE
    // ==========================================

    @Override
    public void init() {

        // Initialize drivetrain.
        drive.init(hardwareMap);
        pollinate.init(hardwareMap);

        // IMPORTANT:
        // Turret must face forward at initialization.
        // Its current encoder position becomes zero.

        // Initialize V2 tracker on red pipeline 1.
        turret.init(
                hardwareMap,
                TurretTrackerV2.RED_PIPELINE);

        turret.setAudienceSide(true);

        telemetry.addLine("Turret V2 Initialized");
        telemetry.addLine("Center turret before INIT");
        telemetry.addLine("X = Blue | B = Red");
        telemetry.addLine("A = Audience | Y = Far");
        telemetry.update();
    }

    // ==========================================
    // BEFORE START
    // ==========================================

    @Override
    public void init_loop() {
pollinate.pollinatorUp();

        // Select blue pipeline 2.
        if (gamepad1.xWasPressed()) {

            redAlliance = false;

            turret.setPipeline(
                    TurretTrackerV2.BLUE_PIPELINE);
        }

        // Select red pipeline 1.
        if (gamepad1.bWasPressed()) {

            redAlliance = true;

            turret.setPipeline(
                    TurretTrackerV2.RED_PIPELINE);
        }

        // Choose audience CELL.
        if (gamepad1.aWasPressed()) {
            audienceSide = true;
            turret.setAudienceSide(true);
        }

        // Choose far CELL.
        if (gamepad1.yWasPressed()) {
            audienceSide = false;
            turret.setAudienceSide(false);
        }

        telemetry.addData(
                "Alliance",
                redAlliance ? "RED" : "BLUE");

        telemetry.addData(
                "Pipeline",
                turret.getPipeline());

        telemetry.addData(
                "CELL",
                audienceSide ? "AUDIENCE" : "FAR");

        telemetry.addData(
                "Turret Encoder",
                turret.getTurretTicks());

        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    // ==========================================
    // START
    // ==========================================

    @Override
    public void start() {

        // Reset timing for the control loop.
        turret.resetTimer();
    }

    // ==========================================
    // MAIN LOOP
    // ==========================================

    @Override
    public void loop() {

        pollinate.pollinatorUp();

        // --------------------------------------
        // DRIVE ROBOT
        // --------------------------------------

        // Keep your original joystick mapping.
        drive.moveBot(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        // --------------------------------------
        // SELECT CELL
        // --------------------------------------

        // A = audience-side CELL.
        if (gamepad1.aWasPressed()) {
            audienceSide = true;
            turret.setAudienceSide(true);
        }

        // Y = far-side CELL.
        if (gamepad1.yWasPressed()) {
            audienceSide = false;
            turret.setAudienceSide(false);
        }

        // --------------------------------------
        // CHANGE TUNING STEP
        // --------------------------------------

        if (gamepad1.bWasPressed()) {
            stepIndex =
                    (stepIndex + 1) % stepSizes.length;
        }

        // --------------------------------------
        // TUNE kP
        // --------------------------------------

        if (gamepad1.dpadLeftWasPressed()) {
            turret.setkP(
                    turret.getkP()
                            - stepSizes[stepIndex]);
        }

        if (gamepad1.dpadRightWasPressed()) {
            turret.setkP(
                    turret.getkP()
                            + stepSizes[stepIndex]);
        }

        // --------------------------------------
        // TUNE kD
        // --------------------------------------

        if (gamepad1.dpadUpWasPressed()) {
            turret.setkD(
                    turret.getkD()
                            + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadDownWasPressed()) {
            turret.setkD(
                    turret.getkD()
                            - stepSizes[stepIndex]);
        }

        // --------------------------------------
        // TUNE PINPOINT FEEDFORWARD
        // --------------------------------------

        // Right bumper makes kFF more positive.
        if (gamepad1.rightBumperWasPressed()) {
            turret.setkFF(
                    turret.getkFF() + FF_STEP);
        }

        // Left bumper makes kFF more negative.
        if (gamepad1.leftBumperWasPressed()) {
            turret.setkFF(
                    turret.getkFF() - FF_STEP);
        }

        // --------------------------------------
        // TUNE RECOVERY GAIN
        // --------------------------------------

        // Detect trigger presses, not continuous holds.
        boolean leftTrigger =
                gamepad1.left_trigger > 0.5;

        boolean rightTrigger =
                gamepad1.right_trigger > 0.5;

        // Right trigger increases recovery gain.
        if (rightTrigger && !previousRightTrigger) {
            turret.setRecoveryKP(
                    turret.getRecoveryKP()
                            + RECOVERY_STEP);
        }

        // Left trigger decreases recovery gain.
        if (leftTrigger && !previousLeftTrigger) {
            turret.setRecoveryKP(
                    turret.getRecoveryKP()
                            - RECOVERY_STEP);
        }

        // Store trigger states for next loop.
        previousLeftTrigger = leftTrigger;
        previousRightTrigger = rightTrigger;

        // --------------------------------------
        // UPDATE TURRET
        // --------------------------------------

        // Reads Limelight and Pinpoint,
        // handles tracking and recovery,
        // and enforces encoder limits.
        turret.update();

        // ======================================
        // TARGET TELEMETRY
        // ======================================

        telemetry.addData(
                "Alliance",
                redAlliance ? "RED" : "BLUE");

        telemetry.addData(
                "Pipeline",
                turret.getPipeline());

        telemetry.addData(
                "CELL",
                audienceSide ? "AUDIENCE" : "FAR");

        telemetry.addData(
                "Cluster",
                turret.getClusterName());

        telemetry.addData(
                "Tags Visible",
                "%d / 4",
                turret.getVisibleTagCount());

        telemetry.addData(
                "Cluster Bearing",
                "%.2f deg",
                turret.getClusterBearing());

        // ======================================
        // V2 TRACKING STATUS
        // ======================================

        telemetry.addLine("---------------------");

        telemetry.addData(
                "TRACKING STATE",
                turret.getTrackingState());

        if (turret.getTrackingState()
                == TurretTrackerV2.TrackingState.TRACKING) {

            telemetry.addLine("Limelight tracking CELL");

        } else if (turret.getTrackingState()
                == TurretTrackerV2.TrackingState.RECOVERING) {

            telemetry.addLine(
                    "Target lost - using Pinpoint memory");

        } else if (turret.getTrackingState()
                == TurretTrackerV2.TrackingState.LIMIT) {

            telemetry.addLine(
                    "TURRET LIMIT - MOTOR BLOCKED");

        } else {

            telemetry.addLine(
                    "No target - turret stopped");
        }

        telemetry.addData(
                "Tracking Error",
                "%.2f deg",
                turret.getError());

        telemetry.addData(
                "Recovery Target Angle",
                "%.2f deg",
                turret.getRecoveryTargetAngle());

        // ======================================
        // PINPOINT / ENCODER TELEMETRY
        // ======================================

        telemetry.addLine("---------------------");

        telemetry.addData(
                "Robot Heading",
                "%.2f deg",
                turret.getRobotHeading());

        telemetry.addData(
                "Robot Yaw Rate",
                "%.2f deg/s",
                turret.getYawRate());

        telemetry.addData(
                "Turret Encoder",
                turret.getTurretTicks());

        telemetry.addData(
                "Turret Angle",
                "%.2f deg",
                turret.getTurretAngleDegrees());

        telemetry.addData(
                "Feedforward Power",
                "%.3f",
                turret.getFFPower());

        telemetry.addData(
                "Turret Motor Power",
                "%.3f",
                turret.getPower());

        // ======================================
        // TUNING VALUES
        // ======================================

        telemetry.addLine("---------------------");

        telemetry.addData(
                "kP (D-Pad Left/Right)",
                "%.5f",
                turret.getkP());

        telemetry.addData(
                "kD (D-Pad Up/Down)",
                "%.6f",
                turret.getkD());

        telemetry.addData(
                "kFF (Bumpers)",
                "%.6f",
                turret.getkFF());

        telemetry.addData(
                "Recovery kP (Triggers)",
                "%.5f",
                turret.getRecoveryKP());

        telemetry.addData(
                "P/D Step (B)",
                "%.7f",
                stepSizes[stepIndex]);

        telemetry.addData(
                "FF Step",
                "%.6f",
                FF_STEP);

        telemetry.update();
    }

    // ==========================================
    // STOP
    // ==========================================

    @Override
    public void stop() {

        // Stop the drivetrain.
        drive.moveBot(0, 0, 0);

        // Stop turret and Limelight.
        turret.stop();
    }
}
