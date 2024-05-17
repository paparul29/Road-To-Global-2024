package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pinza extends SubsystemBase {

    ServoEx servo3;
    ServoEx servoIzq;
    ServoEx servoDer;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Pinza(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
       servoIzq = new SimpleServo(hardwareMap,"servoIzq", 0, 180);
       servoDer = new SimpleServo(hardwareMap,"servoDer", 0, 180);
        servo3 = new SimpleServo(hardwareMap,"servo3", 0, 180);

        servoDer.setInverted(true);


    }


    public void open(){
        servoDer.turnToAngle(70);
        servoIzq.turnToAngle(70);
    }
    public void close(){
        servoDer.turnToAngle(170);
        servoIzq.turnToAngle(110);
    }




    public void grabFundation(){
        servo3.setPosition(1);
    }
    public void leaveFundation(){
        servo3.setPosition(0.5);
    }






}
