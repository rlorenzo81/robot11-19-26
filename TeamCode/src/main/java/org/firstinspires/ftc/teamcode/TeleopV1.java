
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotsystems.Flywheel2;
import org.firstinspires.ftc.teamcode.robotsystems.Intake;
import org.firstinspires.ftc.teamcode.robotsystems.MecanumDriveV4;
import org.firstinspires.ftc.teamcode.robotsystems.Pollinator;

@TeleOp(name = "TeleopV1")
public class TeleopV1 extends OpMode {

    // Subsystems
    MecanumDriveV4 drive = new MecanumDriveV4();
    Flywheel2 fly = new Flywheel2();
    Intake intake = new Intake();
    Pollinator pollinate = new Pollinator();

    // Update telemetry approximately 5 times per second
    private long lastMonitorTime = 0;
    private static final long MONITOR_INTERVAL_NS = 200_000_000L;

    @Override
    public void init() {

        // Pass telemetry to MecanumDrive3
        drive.init(hardwareMap, telemetry);

        fly.init(hardwareMap);
        intake.init(hardwareMap);
        pollinate.init(hardwareMap);
    }

    @Override
    public void loop() {

        // ====================================
        // DRIVETRAIN
        // ====================================

        drive.moveBot(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        // ====================================
        // POLLINATOR
        // ====================================

        if (gamepad1.a) {
            pollinate.pollinatorDown();
        }
        else if (gamepad1.y) {
            pollinate.pollinatorUp();
        }

        // ====================================
        // FLYWHEEL RPM SELECTION
        // ====================================

        if (gamepad2.dpad_up) {
            fly.setTargetRPM(2325);
        }
        else if (gamepad2.dpad_down) {
            fly.setTargetRPM(2500);
        }
        else if (gamepad2.y) {
            fly.setTargetRPM(0);
        }

        // ====================================
        // INTAKE CONTROLS
        // ====================================

        if (gamepad2.a) {
            intake.intakeShoot();
        }
        else if (gamepad2.b) {
            intake.intakeDontShoot();
        }
        else if (gamepad2.left_bumper) {
            intake.stop();
        }
        else if (gamepad2.right_bumper) {
            intake.outtake();
        }

        // ====================================
        // TELEMETRY
        // ====================================

        long now = System.nanoTime();

        if (now - lastMonitorTime >= MONITOR_INTERVAL_NS) {

            lastMonitorTime = now;

            // Read current for each motor
            double lfCurrent = drive.getLFCurrent();
            double lbCurrent = drive.getLBCurrent();
            double rfCurrent = drive.getRFCurrent();
            double rbCurrent = drive.getRBCurrent();

            double totalCurrent =
                    lfCurrent + lbCurrent
                    + rfCurrent + rbCurrent;

            double batteryVoltage = drive.getBatteryVoltage();

            telemetry.addLine("=== DRIVETRAIN ===");

            telemetry.addData(
                    "Battery Voltage",
                    "%.2f V",
                    batteryVoltage
            );

            telemetry.addData(
                    "Filtered Voltage",
                    "%.2f V",
                    drive.getFilteredVoltage()
            );

            telemetry.addData(
                    "LF Current",
                    "%.2f A",
                    lfCurrent
            );

            telemetry.addData(
                    "LB Current",
                    "%.2f A",
                    lbCurrent
            );

            telemetry.addData(
                    "RF Current",
                    "%.2f A",
                    rfCurrent
            );

            telemetry.addData(
                    "RB Current",
                    "%.2f A",
                    rbCurrent
            );

            telemetry.addData(
                    "Total Drive Current",
                    "%.2f A",
                    totalCurrent
            );

            telemetry.addData(
                    "Acceleration Multiplier",
                    "%.1fx",
                    drive.getVoltageMultiplier()
            );

            if (batteryVoltage < 10.5) {
                telemetry.addLine(
                        "WARNING: LOW BATTERY VOLTAGE"
                );
            }

            // ====================================
            // FLYWHEEL TELEMETRY
            // ====================================

            double targetRPM = fly.getTargetRPM();
            double actualRPM = fly.getRPM();
            double rpmError = targetRPM - actualRPM;

            telemetry.addLine("=== FLYWHEEL ===");

            telemetry.addData(
                    "Target RPM",
                    "%.0f",
                    targetRPM
            );

            telemetry.addData(
                    "Actual RPM",
                    "%.0f",
                    actualRPM
            );

            telemetry.addData(
                    "RPM Error",
                    "%.0f",
                    rpmError
            );

            telemetry.addData(
                    "Shooter Ready",
                    fly.atSpeed()
            );

            telemetry.update();
        }
    }

    @Override
    public void stop() {

        drive.stop();
        fly.stopMotors();
        intake.stop();
    }
}
