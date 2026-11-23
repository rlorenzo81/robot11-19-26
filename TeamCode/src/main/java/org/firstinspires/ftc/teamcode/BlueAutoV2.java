
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robotsystems.Flywheel2;
import org.firstinspires.ftc.teamcode.robotsystems.Intake;
import org.firstinspires.ftc.teamcode.robotsystems.Pollinator;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.commands.Commands.*;

@Autonomous(name = "BlueAutoV2")
public class BlueAutoV2 extends OpMode {

    private Follower follower;

    Intake intake = new Intake();
    Flywheel2 fly = new Flywheel2();
    Pollinator pollinator = new Pollinator();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // =====================================
    // BATTERY-FRIENDLY DRIVING SETTINGS
    // =====================================

    // Maximum target acceleration in inches/sec^2
    // Lower = gentler acceleration
    private static final double MAX_ACCELERATION = 35.0;

    // Maximum path speed as a fraction of
    // Pedro's configured achievable speed
    private static final double MAX_PATH_SPEED = 0.80;

    // Collection speed remains slower
    private static final double COLLECTION_SPEED = 0.20;

    // =====================================
    // BLUE POSES
    // ORIGINAL COORDINATES PRESERVED
    // =====================================

    private final Pose startPose =
            poseFactory.of(82.7399, 134.3996, 270);

    private final Pose scorePose =
            poseFactory.of(82.6184, 131.1046, 0);

    private final Pose collectPose =
            poseFactory.of(118, 134, 0);

    private final Pose collectPF =
            poseFactory.of(142, 134, 0);

    private final Pose controlPose =
            poseFactory.of(84.5215, 94.3679, 180);

    private final Pose scoreAgainPose =
            poseFactory.of(127.5894, 15.1386, 144.5);

    private final Pose collectflowerpose =
            poseFactory.of(111.0364, 97.0809, 0);

    private final Pose collectflowerinpose =
            poseFactory.of(130.9221, 97.3073, 0.6211);

    private final Pose scoreflowerpose =
            poseFactory.of(128.0563, 14.8563, 144.5);

    private final Pose controlpose2 =
            poseFactory.of(93.75, 101.13, 139);

    private final Pose parkPose =
            poseFactory.of(127, 48.7322, 90);

    // =====================================
    // PATHS WITH ACCELERATION LIMITING
    // =====================================

    private Path startToScore() {

        return line(startPose, scorePose)
                .linear(startPose, scorePose)
                .with(
                        Constants.foresightConfig
                                .maxAccelerationConstraint
                                .at(MAX_ACCELERATION),

                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(MAX_PATH_SPEED)
                );
    }

    private Path collectPath() {

        return line(scorePose, collectPose)
                .linear(scorePose, collectPose)
                .with(
                        Constants.foresightConfig
                                .maxAccelerationConstraint
                                .at(MAX_ACCELERATION),

                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(MAX_PATH_SPEED)
                );
    }

    private Path collectFP2() {

        return line(collectPose, collectPF)
                .linear(collectPose, collectPF)
                .with(
                        Constants.foresightConfig
                                .maxAccelerationConstraint
                                .at(MAX_ACCELERATION),

                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(COLLECTION_SPEED),

                        Constants.foresightConfig
                                .maxDecelerationScale
                                .at(0.03),

                        Constants.foresightConfig
                                .brakeAggression
                                .at(0.8)
                );
    }

    private Path scoreAgainPath() {

        return curve(
                collectPF,
                controlPose,
                scoreAgainPose
        )
                .linear(collectPF, scoreAgainPose)
                .with(
                        Constants.foresightConfig
                                .maxAccelerationConstraint
                                .at(MAX_ACCELERATION),

                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(MAX_PATH_SPEED)
                );
    }

    private Path flowerPath() {

        return line(scoreAgainPose, collectflowerpose)
                .linear(scoreAgainPose, collectflowerpose)
                .with(
                        Constants.foresightConfig
                                .maxAccelerationConstraint
                                .at(MAX_ACCELERATION),

                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(MAX_PATH_SPEED)
                );
    }

    private Path flowerinPath() {

        return line(collectflowerpose, collectflowerinpose)
                .linear(collectflowerpose, collectflowerinpose)
                .with(
                        Constants.foresightConfig
                                .maxAccelerationConstraint
                                .at(MAX_ACCELERATION),

                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(MAX_PATH_SPEED),

                        Constants.foresightConfig
                                .maxDecelerationScale
                                .at(0.05),

                        Constants.foresightConfig
                                .brakeAggression
                                .at(0.8)
                );
    }

    private Path scoreflowerpath() {

        return curve(
                collectflowerinpose,
                controlpose2,
                scoreflowerpose
        )
                .linear(collectflowerinpose, scoreflowerpose)
                .with(
                        Constants.foresightConfig
                                .maxAccelerationConstraint
                                .at(MAX_ACCELERATION),

                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(MAX_PATH_SPEED)
                );
    }

    private Path park() {

        return line(scoreflowerpose, parkPose)
                .linear(scoreflowerpose, parkPose)
                .with(
                        Constants.foresightConfig
                                .maxAccelerationConstraint
                                .at(MAX_ACCELERATION),

                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(MAX_PATH_SPEED)
                );
    }

    // =====================================
    // FOLLOW TIMEOUT
    // =====================================

    private Command followTimeout(
            Path path,
            long timeoutMs) {

        return race(
                follow(follower, path),
                waitMs(timeoutMs)
        );
    }

    // =====================================
    // AUTONOMOUS ROUTINE
    // ORIGINAL SEQUENCE PRESERVED
    // =====================================

    private Command autoRoutine() {

        return sequential(

                // INITIAL SHOOT
                instant(() -> fly.setTargetRPM(2325)),
                waitMs(2500),

                instant(() -> intake.intakeShoot()),
                waitMs(1500),

                // FIRST SCORE
                followTimeout(startToScore(), 1000),

                instant(() -> intake.intakeDontShoot()),
                instant(() -> fly.setTargetRPM(2500)),

                // FIRST COLLECTION
                followTimeout(collectPath(), 1000),

                followTimeout(collectFP2(), 2300),

                // SECOND SCORE
                instant(() -> fly.setTargetRPM(2400)),

                follow(follower, scoreAgainPath()),

                waitMs(1250),

                instant(() -> intake.intakeShoot()),
                waitMs(1250),

                // FLOWER COLLECTION
                followTimeout(flowerPath(), 1000),

                instant(() -> pollinator.pollinatorDown()),
                instant(() -> intake.intakeDontShoot()),

                followTimeout(flowerinPath(), 1000),

                waitMs(2000),

                instant(() -> fly.setTargetRPM(2400)),

                // THIRD SCORE
                follow(follower, scoreflowerpath()),

                waitMs(1250),

                instant(() -> intake.intakeShoot()),
                waitMs(1250),

                instant(() -> pollinator.pollinatorUp()),

                // PARK
                instant(() -> intake.intakeDontShoot()),

                followTimeout(park(), 1000)
        );
    }

    // =====================================
    // INIT
    // =====================================

    @Override
    public void init() {

        fly.init(hardwareMap);
        intake.init(hardwareMap);
        pollinator.init(hardwareMap);

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);
        follower.update();

        telemetry.addLine("BlueAutoV2 Initialized");

        telemetry.addData(
                "Max Acceleration",
                "%.1f in/s^2",
                MAX_ACCELERATION
        );

        telemetry.addData(
                "Max Path Speed",
                "%.0f%%",
                MAX_PATH_SPEED * 100
        );

        telemetry.update();
    }

    // =====================================
    // START
    // =====================================

    @Override
    public void start() {

        schedule(autoRoutine());
    }

    // =====================================
    // LOOP
    // =====================================

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        telemetry.addData(
                "X",
                follower.pose().x()
        );

        telemetry.addData(
                "Y",
                follower.pose().y()
        );

        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );

        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        telemetry.addData(
                "Path Completion",
                follower.completion()
        );

        telemetry.addData(
                "Distance Remaining",
                follower.remainingDistance()
        );

        telemetry.addData(
                "Acceleration Limit",
                "%.1f in/s^2",
                MAX_ACCELERATION
        );

        telemetry.addData(
                "Normal Path Speed",
                "%.0f%%",
                MAX_PATH_SPEED * 100
        );

        telemetry.update();
    }

    // =====================================
    // STOP
    // =====================================

    @Override
    public void stop() {

        if (follower != null) {
            OpModeStorage.autonomousEndPose = follower.pose();
        }
    }
}
