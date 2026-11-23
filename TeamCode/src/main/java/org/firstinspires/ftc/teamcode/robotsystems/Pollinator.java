package org.firstinspires.ftc.teamcode.robotsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pollinator {
    public Servo leftPollinator;
    public Servo rightPollinator;

    public void init(HardwareMap hwMap) {

        leftPollinator= hwMap.get(Servo.class, "lp");
        rightPollinator= hwMap.get(Servo.class, "rp");

      leftPollinator.setPosition(0.6);
      rightPollinator.setPosition(0.4);
    }

    public void pollinatorDown(){
        leftPollinator.setPosition(0.2);
        rightPollinator.setPosition(0.8);

    }

    public void pollinatorUp(){
        leftPollinator.setPosition(0.6);
        rightPollinator.setPosition(0.4);

    }
}
