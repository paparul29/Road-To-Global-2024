package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pinza extends SubsystemBase {

    ServoEx servoIzq;
    ServoEx servoDer;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    public Pinza(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
       servoIzq = new SimpleServo(hardwareMap,"servoIzq", 0, 180);
       servoDer = new SimpleServo(hardwareMap,"servoDer", 0, 180);


        servoIzq.setInverted(false);


    }


    public void close(){
        servoDer.setPosition(1);
        servoIzq.setPosition(0);
    }
    public void open(){
        servoDer.setPosition(.9);
        servoIzq.setPosition(0.32);
    }













}
