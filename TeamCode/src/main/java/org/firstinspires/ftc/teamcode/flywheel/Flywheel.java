package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private DcMotorEx shooter;

    private double encoderCPM= 28;

    private double gearRatio = 3;

    private double targetRPM = 0;

    private double kV=0.000537,kS=0.09,kP=0.0058;

    public void init(HardwareMap hwMap){
        shooter = hwMap.get(DcMotorEx.class, "shoot");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setMotorPower(double power){

        shooter.setPower(power);
    }

    public void stopMotors() {

        shooter.setPower(0);
    }

    public void setMotorRPM(double targetRpm){
        //call this every loop
        double error = targetRpm - getRPM();
        double ff = (kV * targetRpm) + kS;
        double fb = error*kP;
        double power = ff+fb;

        setMotorPower(power);
    }

    public double getTargetRPM() {
        return targetRPM;
    }
    public double getTicksPerSec(){
        return shooter.getVelocity();
    }

    public double getRPM(){
        return ((getTicksPerSec()/encoderCPM) * 60) / gearRatio;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    public void update() {
        if (targetRPM > 0) {
            setMotorRPM(targetRPM);
        } else {
            stopMotors();
        }
    }




}
