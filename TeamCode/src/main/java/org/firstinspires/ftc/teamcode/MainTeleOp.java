package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.robotsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotsystems.Intake;

@Disabled

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends OpMode {

    // Create drivetrain subsystem
    MecanumDrive drive = new MecanumDrive();
    Flywheel fly = new Flywheel();

    Intake intake = new Intake();

    @Override
    public void init() {

        // Initialize drivetrain hardware
        drive.init(hardwareMap);
        fly.init(hardwareMap);
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {

        // Drive using gamepad joysticks
        drive.moveBot(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        // Flywheel RPM selection
        if (gamepad2.dpad_up) {
            fly.setTargetRPM(775);
        }

        if (gamepad2.dpad_up) {
            fly.setTargetRPM(800);
        }

        if(gamepad2.a){
            intake.intakeShoot();
        }



        else if(gamepad2.b){
            intake.intakeDontShoot();
        }


        else if(gamepad2.left_bumper){
            intake.stop();

        }



        fly.update();

        // Telemetry
        telemetry.addData("Target RPM", fly.getTargetRPM());
        telemetry.addData("Actual RPM", fly.getRPM());
        telemetry.update();
    }

    @Override
    public void stop() {

        drive.stop();
        fly.stopMotors();
        intake.stop();
    }

}
