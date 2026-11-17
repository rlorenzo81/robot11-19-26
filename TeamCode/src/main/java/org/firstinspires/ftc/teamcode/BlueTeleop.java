
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotsystems.Flywheel2;
import org.firstinspires.ftc.teamcode.robotsystems.Intake;
import org.firstinspires.ftc.teamcode.robotsystems.MecanumDrive;

@TeleOp(name = "Blue Teleop")
public class BlueTeleop extends OpMode {

    // Subsystems
    MecanumDrive drive = new MecanumDrive();
    Flywheel2 fly = new Flywheel2();
    Intake intake = new Intake();

    @Override
    public void init() {

        drive.init(hardwareMap);
        fly.init(hardwareMap);
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {

        // Drivetrain
        drive.moveBot(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        // Flywheel RPM selection
        if (gamepad2.dpad_up) {
            fly.setTargetRPM(2325);
        }
        else if (gamepad2.dpad_down) {
            fly.setTargetRPM(2450);
        }
        else if (gamepad2.y){
            fly.setTargetRPM(0);
        }

        // Intake controls
        if (gamepad2.a) {
            intake.intakeShoot();
        }
        else if (gamepad2.b) {
            intake.intakeDontShoot();
        }
        else if (gamepad2.left_bumper) {
            intake.stop();
        }



        // Flywheel telemetry
        double targetRPM = fly.getTargetRPM();
        double actualRPM = fly.getRPM();
        double rpmError = targetRPM - actualRPM;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("RPM Error", rpmError);
        telemetry.addData("Shooter Ready", fly.atSpeed());

        telemetry.update();
    }

    @Override
    public void stop() {

        drive.stop();
        fly.stopMotors();
        intake.stop();
    }
}
