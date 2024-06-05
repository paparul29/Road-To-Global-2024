package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Roller extends SubsystemBase {
    DcMotorEx roller;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Roller(Telemetry telemetry, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        roller = hardwareMap.get(DcMotorEx.class, "roller");
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //roller.setDirection(DcMotor.Direction.FORWARD);
        //roller.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setRoller(double power){
        roller.setPower(power);
    }
   /* public void get(){
        roller.setPower(.9);
    }
*/
    public void stop(){
        roller.setPower(0);
    }
    public void out() {
        roller.setPower(1);
    }
}
