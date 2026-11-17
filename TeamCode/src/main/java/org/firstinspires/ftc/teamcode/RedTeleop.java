
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotsystems.Flywheel2;
import org.firstinspires.ftc.teamcode.robotsystems.Intake;
import org.firstinspires.ftc.teamcode.robotsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotsystems.Pollinator;

@TeleOp(name = "Red Teleop")
public class RedTeleop extends OpMode {

    // Subsystems
    MecanumDrive drive = new MecanumDrive();
    Flywheel2 fly = new Flywheel2();
    Intake intake = new Intake();
    Pollinator pollinate =new Pollinator();

    @Override
    public void init() {

        drive.init(hardwareMap);
        fly.init(hardwareMap);
        intake.init(hardwareMap);
        pollinate.init(hardwareMap);
    }

    @Override
    public void loop() {

        // Drivetrain
        drive.moveBot(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        if(gamepad1.a){
            pollinate.pollinatorDown();
        }

        else if(gamepad1.y){
            pollinate.pollinatorUp();
        }

        // Flywheel RPM selection
        if (gamepad2.dpad_up) {
            fly.setTargetRPM(2325);
        }
        else if (gamepad2.dpad_down) {
            fly.setTargetRPM(2500);
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

        else if (gamepad2.right_bumper){
            intake.outtake();
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
