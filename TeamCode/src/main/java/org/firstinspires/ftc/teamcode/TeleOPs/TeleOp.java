package org.firstinspires.ftc.teamcode.TeleOPs;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Commands.RollerCommand;
import org.firstinspires.ftc.teamcode.Commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Brazo;
import org.firstinspires.ftc.teamcode.Subsystems.Escalador;
import org.firstinspires.ftc.teamcode.Subsystems.Pinza;
import org.firstinspires.ftc.teamcode.Subsystems.Roller;
import org.firstinspires.ftc.teamcode.Subsystems.TankSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        GamepadEx gamepadDriver = new GamepadEx(gamepad1);
        GamepadEx gamepadsystem = new GamepadEx(gamepad2);

        SampleTankDrive sampleTankDrive = new SampleTankDrive(hardwareMap);
        TankSubsystem driveSystem = new TankSubsystem(sampleTankDrive);
        Brazo brazo= new Brazo(telemetry,hardwareMap);
        Pinza pinza = new Pinza(telemetry, hardwareMap);
        //Escalador escalador = new Escalador(hardwareMap,telemetry);
        //Roller roller = new Roller(telemetry,hardwareMap);

        driveSystem.setDefaultCommand(new TankDriveCommand(
                driveSystem, () -> -gamepadDriver.getLeftY(), gamepadDriver::getRightX
        ));

        schedule(new RunCommand(() -> {
            driveSystem.update();
            telemetry.addData("Heading", driveSystem.getPoseEstimate().getHeading());
            telemetry.update();
        }));



        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(pinza::open));
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(pinza::close));




        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.Y)
                .whileHeld(()-> brazo.poder(.4))
                .whenReleased(()->brazo.poder(0));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.A)
                .whileHeld(()-> brazo.poder(-.4))
                .whenReleased(()->brazo.poder(0));





        new GamepadButton(new GamepadEx(gamepad2),GamepadKeys.Button.B)
                .toggleWhenPressed(()-> pinza.grabFundation(),()-> pinza.leaveFundation());




      /* new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.Y)
                .whenPressed(() -> brazo.posiciones(-700, .5));



        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.A)
                .whenPressed(() -> brazo.posiciones(0, .5));



        /*new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.Y)
                .whenPressed(()->escalador.setPower(.8));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.A)
                .whenPressed(()->escalador.setPower(-.8));

         */


        /*new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(roller::setRoller))
                .whenReleased(new InstantCommand(roller::stop));


        new GamepadButton(new GamepadEx(gamepad1),GamepadKeys.Button.LEFT_BUMPER)
              .whenPressed(new InstantCommand(roller::out))
            .whenReleased(new InstantCommand(roller::stop));


         */





       // roller.setDefaultCommand(new RollerCommand(roller,gamepadDriver));


        gamepadDriver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(driveSystem::toggleInverted));



    }


}
