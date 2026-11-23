
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

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.paths.Path;

@Autonomous(name = "RedFront")
public class RedFront extends OpMode {

    private Follower follower;
    Intake intake = new Intake();
    Flywheel2 fly = new Flywheel2();

    Pollinator pollinator = new Pollinator();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // POSES - COORDENADAS
    private final Pose startPose = poseFactory.of(61.2601, 9.6004, 90);
    private final Pose scorePose = poseFactory.of(61.3816, 12.8954, 180);
    private final Pose collectPose = poseFactory.of(26, 10, 180);
    private final Pose parkPose = poseFactory.of(17, 110.2678, 270);

    // PATHS

    private Path startToScore() {
        return line(startPose, scorePose)
                .linear(startPose, scorePose);
    }
    private Path collectPath() {
        return line(scorePose, collectPose)
                .linear(scorePose, collectPose);
    }
    //private Path collectFP() {
    //   return line(collectPose, collectPF)
    // .linear(collectPose, collectPF)
    // .with(
    //          Constants.foresightConfig.maxDecelerationScale.at(0.1),
    //         Constants.foresightConfig.brakeAggression.at(0.8)
    //   );
    // }

    private Path park() {
        return line(collectPose, parkPose)
                .linear(collectPose, parkPose);
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

    // AUTONOMOUS ROUTINE

    private Command autoRoutine() {
        return sequential(

                instant(() -> fly.setTargetRPM(2325)),
                waitMs(2500), //3000
                instant(() -> intake.intakeShoot()),
                waitMs(1500),

                // Ir al primer SCORE
                followTimeout(startToScore(), 1000),
                instant(() -> intake.intakeDontShoot()),
                instant(() -> fly.setTargetRPM(2500)),

                // Ir a COLLECT
                followTimeout(collectPath(), 1000),

                //  followTimeout(collectFP(), 1000),

                followTimeout(park(), 1000));
    }

    // =========================
    // INIT
    // =========================

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

    // =========================
    // START
    // =========================

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    // =========================
    // LOOP
    // =========================

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
