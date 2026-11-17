package org.firstinspires.ftc.teamcode.pedro.procedures;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

public class TeleOP {private Follower follower;

    public void init() {
        follower = Constants.create(hardwareMap);
    }


    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }
public void loop() {
    DrivePowers powers = ManualDrive.fieldCentric(
            -gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            gamepad1.right_stick_x,
            follower.pose().heading()
    );
    follower.manual(powers);

    // relocalise button
    if (gamepad1.start ) {
        Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
        // On the fly Pose creation, we don't recommend this for Autonomous. Only accepts radians for heading
        follower.setPose(cornerPose); // overrides our pose
    }

    follower.update();
    Pose robotPose = follower.pose(); // returns a Pose object
    telemetry.addData("Robot X", robotPose.x());
    telemetry.addData("Robot Y", robotPose.y());
    telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    // Math.toDegrees() is a built-in java method
}}