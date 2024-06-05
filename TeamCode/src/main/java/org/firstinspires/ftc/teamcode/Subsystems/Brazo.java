package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Brazo extends SubsystemBase {

   DcMotorEx brazo;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Brazo(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        brazo = hardwareMap.get(DcMotorEx.class,"brazo1");//poder
        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//poder
        brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    /*public void poder(double power){
        brazo.setPower(power);
   }

     */



   public int getArmPose(){
        return brazo.getCurrentPosition();
   }






   public void posiciones(int pos, double power){
        brazo.setTargetPosition(pos);
        brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brazo.setPower(power);

    }









    @Override
    public void periodic() {
        telemetry.addData("Brazo: ", brazo.getCurrentPosition());
    }








}
