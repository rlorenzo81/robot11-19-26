package org.firstinspires.ftc.teamcode.turrettracking;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robotsystems.Flywheel2;
import org.firstinspires.ftc.teamcode.robotsystems.Intake;
import org.firstinspires.ftc.teamcode.robotsystems.Pollinator;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;

import static com.pedropathing.api.Paths.*;
        import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import static com.pedropathing.ivy.commands.Commands.*;

        import static java.lang.invoke.MethodHandles.constant;

import com.pedropathing.paths.Path;


@Autonomous(name = "RedAuto2")
public class RedAuto2 extends OpMode {

    private Follower follower;
    Intake intake = new Intake();
    Flywheel2 fly = new Flywheel2();

    Pollinator pollinator = new Pollinator();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // =========================
    // POSES
    // =========================

    private final Pose startPose = poseFactory.of(61, 0, 90);
    private final Pose scorePose = poseFactory.of(61, 5, 90);
    private final Pose collectPose = poseFactory.of(26, 4, 180);
    private final Pose collectPF = poseFactory.of(13.5, 0.8, 182.5);

    private final Pose controlPose = poseFactory.of(50, 40, 280);
    private final Pose scoreAgainPose = poseFactory.of(25, 120, 315);
   private final Pose collectflowerpose = poseFactory.of(30, 48, 180);
    private final Pose parkPose = poseFactory.of(15, 120, 315);

    // =========================
    // PATHS
    // =========================

    private Path startToScore() {
        return line(startPose, scorePose)
                .linear(startPose, scorePose);
    }

    private Path collectPath() {
        return line(scorePose, collectPose)
                .linear(scorePose, collectPose);
    }

    private Path collectFP() {
        return line(collectPose, collectPF)
                .linear(collectPose, collectPF)
                .with(
                        Constants.foresightConfig.maxDecelerationScale.at(0.1),
                        Constants.foresightConfig.brakeAggression.at(0.8)
                );
    }

    private Path collectFP2() {
        return line(collectPose, collectPF)
                .linear(collectPose, collectPF)
                .with(
                        Constants.foresightConfig.maxDecelerationScale.at(0.1),
                        Constants.foresightConfig.brakeAggression.at(0.8)
                );
    }

    private Path scoreAgainPath() {
        return curve(collectPF,controlPose, scoreAgainPose)
                .linear(collectPF, scoreAgainPose);
    }
    private Path flowerPath() {
        return line(scoreAgainPose, collectflowerpose)
                .linear(scoreAgainPose, collectflowerpose);
    }

    private Path park() {
        return line(collectflowerpose, parkPose)
                .linear(collectflowerpose, parkPose);
    }

    // =========================
    // FOLLOW TIMEOUT
    // =========================

    private Command followTimeout(Path path, long timeoutMs) {
        return race(
                follow(follower, path),
                waitMs(timeoutMs)
        );
    }

    // =========================
    // AUTONOMOUS ROUTINE
    // =========================

    private Command autoRoutine() {
        return sequential(
                instant(() -> fly.setTargetRPM(2325)),
                waitMs(3500),
                instant(() -> intake.intakeShoot()),
                waitMs(1500),

                // Ir al primer SCORE
                followTimeout(startToScore(), 1000),

                instant(() -> intake.intakeDontShoot()),
                instant(() -> fly.setTargetRPM(2500)),

                // Ir a COLLECT
                followTimeout(collectPath(), 1000),

                followTimeout(collectFP(), 1000),

                followTimeout(collectFP2(), 1000),

                // Regresar al segundo SCORE
                followTimeout(scoreAgainPath(), 1000),

                waitMs(1000),

                instant(() -> intake.intakeShoot()),
                waitMs(1500),
                instant(() -> intake.intakeDontShoot()),
followTimeout(flowerPath(), 1000),
                instant(() -> intake.intakeShoot()),

                // Ir a PARK
                followTimeout(park(), 1000),
                instant(() -> intake.intakeDontShoot())

        );
    }



    @Override
    public void init() {
        fly.init(hardwareMap);
        intake.init(hardwareMap);
        pollinator.init(hardwareMap);
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);
        follower.update();

        telemetry.addLine("Autonomous Initialized");
        telemetry.update();
    }


    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {

        //fly.setTargetRPM(2325);//was2450

        follower.update();

        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());

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

        telemetry.update();
    }

    // =========================
    // STOP
    // =========================

    @Override
    public void stop() {

        if (follower != null) {
            OpModeStorage.autonomousEndPose = follower.pose();
        }
    }
}