package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BrazoCore extends SubsystemBase {


    DcMotorEx armMotor1;

    DcMotorEx armMotor2;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    //TWO NEGATIVE

    public BrazoCore(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        armMotor1 = hardwareMap.get(DcMotorEx.class,"brazo1");
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor2 = hardwareMap.get(DcMotorEx.class,"brazo2");
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }



    public void setPosition(int pos, double power){
        armMotor1.setTargetPosition(pos);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1.setPower(power);


        armMotor2.setTargetPosition(-pos);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setPower(power);

    }

    public void resetArmTicks(){
        armMotor1.setPower(0);
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor2.setPower(0);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public int  getArm1Pose(){
        return armMotor1.getCurrentPosition();
    }

    public int getArm2Pose()
    {
        return armMotor2.getCurrentPosition();
    }

    @Override
    public void periodic(){
        telemetry.addData("brazo1",armMotor1.getCurrentPosition());
        telemetry.addData("Tps 1",armMotor1.getVelocity());

        telemetry.addData("brazo2",armMotor2.getCurrentPosition());
        telemetry.addData("Tps 2",armMotor2.getVelocity());
    }


}

