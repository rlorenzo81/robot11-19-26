package org.firstinspires.ftc.teamcode.robotsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx rightFront = null;


    public void init(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotorEx.class, "lf");
        leftBack = hwMap.get(DcMotorEx.class, "lb");
        rightBack= hwMap.get(DcMotorEx.class, "rb");
        rightFront= hwMap.get(DcMotorEx.class, "rf");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define and initialize ALL installed servos.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
stop();
    }

    // Mecanum drive
    // Mecanum drive
    public void moveBot(double x, double y, double turn) {

        double denominator = Math.max(
                Math.abs(x) + Math.abs(y) + Math.abs(turn),
                1.0
        );

        double lf = (x - y + turn) / denominator;
        double lb = (-x - y + turn) / denominator;
        double rf = (-x - y - turn) / denominator;
        double rb = (x - y - turn) / denominator;

        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    // Stop all drive motors
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}


