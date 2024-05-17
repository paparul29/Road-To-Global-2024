package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.TankSubsystem;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends CommandBase {

    private final TankSubsystem drive;
    private final DoubleSupplier leftY;
    private final DoubleSupplier rightX;

    public TankDriveCommand(TankSubsystem drive, DoubleSupplier leftY, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftY = rightX;
        this.rightX = leftY;

        addRequirements(drive);
    }

    @Override
    public void execute(){
        drive.drive(leftY.getAsDouble(), rightX.getAsDouble());
    }

}
