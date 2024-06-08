package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.BrazoHD;

public class BrazoRunn extends CommandBase {

    BrazoHD brazo;
    int setpoint;

    public BrazoRunn(BrazoHD brazo, int setpoint){
        this.brazo =  brazo;
        this.setpoint = setpoint;

        addRequirements(brazo);
    }

    @Override
    public void execute (){
    }



}
