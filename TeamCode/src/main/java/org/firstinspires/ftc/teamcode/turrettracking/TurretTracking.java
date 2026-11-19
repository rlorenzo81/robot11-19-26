package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TurretTracking {

    private DcMotorEx turret;

    private double kP =0.0001;
    private double kD =0.0000;

    private double goalX =0;
    private double lastError=0;
    private double targetTolerance = 0.2;

    private double angleTolerance = 0.2;
    private final double MAX_POWER=0.6;
    private double power =0;
    private final ElapsedTime timer =  new ElapsedTime();

    private void init(HardwareMap hwMap){
        turret=hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
public void setkP(double newKP)
{
    kP=newKP;
}

public double getKP(){
        return kP;
}

public void setkD(double newKD)
    {
        kD=newKD;
    }
    public double getKD(){
        return kD;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void update (AprilTagDetection curID) {
        double deltaTime = timer.seconds();
        timer.reset();

        if(curID == null) {
            turret.setPower(0);
            lastError=0;
            return;
    }
        // -------- start PD controller ------

        double error =goalX - curID.ftcPose.bearing;
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error-lastError)/deltaTime)*kD;
        }
       if (Math.abs(error) < angleTolerance) {
           power=0;
       } else {
           power= Range.clip(pTerm+dTerm, -MAX_POWER, MAX_POWER);
       }

       // saftey encoder check  OR magnetic limit switch check / other saftiesx
        turret.setPower(power);
       lastError=error;
        }
}
