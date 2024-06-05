package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;

public class RollerCommand extends CommandBase {
   Roller roller;
    GamepadEx gamepad;

    public RollerCommand(Roller roller, GamepadEx gamepad){
        this.roller = roller;
        this.gamepad = gamepad;

        addRequirements(roller);
    }

    @Override
    public void execute() {
        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)> .9){
            roller.setRoller(1);
        }else if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)> 0){
            roller.setRoller(0);
        }

        if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)> .9){
            roller.setRoller(-1);
        }else if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)> 0){
            roller.setRoller(0);
        }
    }




}
