package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robotsystems.Flywheel2;
import org.firstinspires.ftc.teamcode.robotsystems.Intake;


@Autonomous(name = "Base Auto")
public class BaseAuto extends OpMode {

    private Follower follower;
    Intake intake = new Intake();
    Flywheel2 fly =new Flywheel2();


    private final PoseFactory poseFactory = PoseFactory.degrees();

    // =========================
    // POSES
    // =========================

    private final Pose startPose = poseFactory.of(61, 0, 90);
    private final Pose scorePose = poseFactory.of(61, 5, 90);
    private final Pose collectPose = poseFactory.of(13.5, 1.4, 180);
    private final Pose scoreAgainPose = poseFactory.of(61, 5, 90);
    private final Pose parkPose = poseFactory.of(30, 5, 180);

    // =========================
    // PATHS
    // =========================

    private Path startToScore() {
        return line(startPose, scorePose)
                .linear(startPose, scorePose);
    }

    private Path collectPath() {
        return line(scorePose, collectPose)
                .linear(scorePose, collectPose)
        .with(
                Constants.foresightConfig.maxDecelerationScale.at(0.1), // Frena más suave y empieza a reducir velocidad antes
                Constants.foresightConfig.brakeAggression.at(0.8)        // Opcional: evita que se pase del punto final (default es 1.0)
        );
    }

    private Path scoreAgainPath() {
        return line(collectPose, scoreAgainPose)
                .linear(collectPose, scoreAgainPose);


    }

    private Path park() {
        return line(scoreAgainPose, parkPose)
                .linear(scoreAgainPose, parkPose);
    }

    // =========================
    // AUTONOMOUS ROUTINE
    // =========================

    private Command autoRoutine() {
        return sequential(
waitMs(3000),
                instant(() ->intake.intakeShoot()),
                waitMs(1500),

                // Ir al primer SCORE

                follow(follower, startToScore()),


                // PAUSA DE 5 SEGUNDOS


                instant(() ->intake.intakeDontShoot()),

                // Ir a COLLECT
                follow(follower, collectPath()),

                // Regresar al segundo SCORE


                follow(follower, scoreAgainPath()),
                waitMs(1000),
                instant(() ->intake.intakeShoot()),
                waitMs(1500),
                // PAUSA DE 5 SEGUNDOS


                // Ir a PARK
                follow(follower, park())
        );
    }

    // =========================
    // INIT
    // =========================

    @Override
    public void init() {
        fly.init(hardwareMap);
        intake.init(hardwareMap);

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

        fly.setTargetRPM(2325);//was2490

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