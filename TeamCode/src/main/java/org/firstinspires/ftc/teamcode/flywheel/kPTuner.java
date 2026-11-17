package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class kPTuner extends OpMode {
    Flywheel flywheel =new Flywheel();

    public static double kV=0.000537; //0.000470
public double kS=0.09;
public double kP=0.0058;
public double goalRPM = 750;
    double[] increments = {0.000001,0.00001,0.0001,0.001,0.01};
    int incrementIdx = 2; //start at 0.01

    @Override
    public void init (){
        flywheel.init(hardwareMap);
    }

    @Override
    public void loop(){
        if(gamepad1.dpadRightWasPressed()&& incrementIdx<4){
            incrementIdx++;
        } else if (gamepad1.dpadLeftWasPressed() && incrementIdx > 0){
            incrementIdx--;
        }

        if(gamepad1.a){
            goalRPM=750;
        }
        else if (gamepad1.b){
            goalRPM = 1000;
        }

        double currentStep = increments[incrementIdx];
        // change kS
        if (gamepad1.dpadUpWasPressed()) {kP += currentStep;}
        if (gamepad1.dpadDownWasPressed()) {kP -= currentStep;}

        double feedForward = (kV*goalRPM)+kS;
        double error = goalRPM - flywheel.getRPM();
        double feedBack = error*kP;

        flywheel.setMotorPower(feedForward+feedBack);

        telemetry.addData("Step","%.6f", currentStep);
        telemetry.addData("kP","%.6f",kP);
        telemetry.addData("Error", error);
        telemetry.addData("RPM",flywheel.getRPM());
    }
}
