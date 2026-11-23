
package org.firstinspires.ftc.teamcode.turrettracking;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotsystems.MecanumDrive;
@Disabled
@TeleOp(name = "Turret Pinpoint Tracking Tuner")
public class TurretAutoAliginOpModeTutorial extends OpMode {

    MecanumDrive drive = new MecanumDrive();

    // Main turret tracking mechanism.
    private final TurretMechanismTutorial turret =
            new TurretMechanismTutorial();

    // =====================================
    // ALLIANCE SELECTION
    // =====================================

    private boolean redAlliance = true;
    private boolean audienceSide = true;

    // =====================================
    // TUNING STEPS
    // =====================================

    // Existing P and D adjustment sizes.
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

    // Separate small step for feedforward.
    private static final double FF_STEP = 0.0001;

    // =====================================
    // INITIALIZE
    // =====================================

    @Override
    public void init() {

        drive.init(hardwareMap);

        // Start with red pipeline 1.
        turret.init(
                hardwareMap,
                TurretMechanismTutorial.RED_PIPELINE);

        // Start with audience-side cluster.
        turret.setAudienceSide(true);

        telemetry.addLine("Turret / Pinpoint Initialized");
        telemetry.addLine("X = Blue | B = Red");
        telemetry.addLine("A = Audience | Y = Far");
        telemetry.update();
    }

    // Called repeatedly before START.
    @Override
    public void init_loop() {

        // Choose the blue Limelight pipeline.
        if (gamepad1.xWasPressed()) {

            redAlliance = false;

            turret.setPipeline(
                    TurretMechanismTutorial.BLUE_PIPELINE);
        }

        // Choose the red Limelight pipeline.
        if (gamepad1.bWasPressed()) {

            redAlliance = true;

            turret.setPipeline(
                    TurretMechanismTutorial.RED_PIPELINE);
        }

        // Select audience-side CELL.
        if (gamepad1.aWasPressed()) {
            audienceSide = true;
            turret.setAudienceSide(true);
        }

        // Select far-side CELL.
        if (gamepad1.yWasPressed()) {
            audienceSide = false;
            turret.setAudienceSide(false);
        }

        // Show configuration.
        telemetry.addData(
                "Alliance",
                redAlliance ? "RED" : "BLUE");

        telemetry.addData(
                "Pipeline", turret.getPipeline());

        telemetry.addData(
                "CELL",
                audienceSide ? "AUDIENCE" : "FAR");

        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    // =====================================
    // START
    // =====================================

    @Override
    public void start() {

        // Reset PD loop timing.
        turret.resetTimer();
    }

    // =====================================
    // MAIN LOOP
    // =====================================

    @Override
    public void loop() {

        // Drivetrain
        drive.moveBot(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        // ---------------------------------
        // SELECT WHICH CELL TO TRACK
        // ---------------------------------

        if (gamepad1.aWasPressed()) {
            audienceSide = true;
            turret.setAudienceSide(true);
        }

        if (gamepad1.yWasPressed()) {
            audienceSide = false;
            turret.setAudienceSide(false);
        }

        // ---------------------------------
        // CHANGE TUNING STEP SIZE
        // ---------------------------------

        if (gamepad1.bWasPressed()) {
            stepIndex =
                    (stepIndex + 1) % stepSizes.length;
        }

        // ---------------------------------
        // TUNE kP
        // ---------------------------------

        if (gamepad1.dpadLeftWasPressed()) {
            turret.setkP(
                    turret.getkP() - stepSizes[stepIndex]);
        }

        if (gamepad1.dpadRightWasPressed()) {
            turret.setkP(
                    turret.getkP() + stepSizes[stepIndex]);
        }

        // ---------------------------------
        // TUNE kD
        // ---------------------------------

        if (gamepad1.dpadUpWasPressed()) {
            turret.setkD(
                    turret.getkD() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadDownWasPressed()) {
            turret.setkD(
                    turret.getkD() - stepSizes[stepIndex]);
        }

        // ---------------------------------
        // TUNE PINPOINT FEEDFORWARD
        // ---------------------------------

        // Right bumper increases kFF.
        if (gamepad1.rightBumperWasPressed()) {
            turret.setkFF(
                    turret.getkFF() + FF_STEP);
        }

        // Left bumper decreases kFF.
        if (gamepad1.leftBumperWasPressed()) {
            turret.setkFF(
                    turret.getkFF() - FF_STEP);
        }

        // ---------------------------------
        // UPDATE TRACKING
        // ---------------------------------

        // Read Pinpoint and Limelight,
        // then update the turret motor.
        turret.update();

        // =====================================
        // TARGET TELEMETRY
        // =====================================

        telemetry.addData(
                "Alliance",
                redAlliance ? "RED" : "BLUE");

        telemetry.addData(
                "Pipeline",
                turret.getPipeline());

        telemetry.addData(
                "Selected CELL",
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

        telemetry.addData(
                "Angle Error",
                "%.2f deg",
                turret.getError());

        telemetry.addData(
                "Turret Encoder",
                turret.getTurretTicks());

        // Show whether camera sees the CELL.
        if (turret.isTargetVisible()) {
            telemetry.addLine("Target: VISIBLE");
        } else if (turret.isUsingLossGrace()) {
            telemetry.addLine("Target: SHORT LOSS");
        } else {
            telemetry.addLine("Target: LOST - STOPPED");
        }

        // Warn when a configured limit is reached.
        if (turret.isAtEncoderLimit()) {
            telemetry.addLine("ENCODER LIMIT REACHED");
        }

        // =====================================
        // PINPOINT TELEMETRY
        // =====================================

        telemetry.addLine("---------------------");

        telemetry.addData(
                "Robot Yaw Rate",
                "%.2f deg/s",
                turret.getYawRate());

        telemetry.addData(
                "Feedforward Power",
                "%.3f",
                turret.getFFPower());

        telemetry.addData(
                "Turret Motor Power",
                "%.3f",
                turret.getPower());

        // =====================================
        // TUNING TELEMETRY
        // =====================================

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
                "P/D Step (B)",
                "%.7f",
                stepSizes[stepIndex]);

        telemetry.addData(
                "FF Step",
                "%.6f",
                FF_STEP);

        telemetry.update();
    }

    // =====================================
    // STOP
    // =====================================

    @Override
    public void stop() {

        // Stop turret and Limelight.
        turret.stop();
    }
}
