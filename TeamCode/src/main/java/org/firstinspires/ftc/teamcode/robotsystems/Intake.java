package org.firstinspires.ftc.teamcode.robotsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {

    public DcMotor frontIntake = null;
    public DcMotor shootIntake = null;

    public void init(HardwareMap hwMap) {

        frontIntake = hwMap.get(DcMotorEx.class, "fi");
        shootIntake = hwMap.get(DcMotorEx.class, "si");

        frontIntake.setPower(0);
        shootIntake.setPower(0);
    }

    // Gamepad A behavior
    public void intakeShoot() {
        frontIntake.setPower(1);
        shootIntake.setPower(-1);
    }

    // Gamepad B behavior
    public void intakeDontShoot() {
        frontIntake.setPower(1);
        shootIntake.setPower(1);
    }
    public void outtake() {
        frontIntake.setPower(-1);
        shootIntake.setPower(1);
    }

    // Stop both intake motors
    public void stop() {
        frontIntake.setPower(0);
        shootIntake.setPower(0);
    }
}
