package org.firstinspires.ftc.teamcode.TeleOPs;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.BrazoHD;
import org.firstinspires.ftc.teamcode.Subsystems.Pinza;
import org.firstinspires.ftc.teamcode.Subsystems.TankSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

//@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpHD extends CommandOpMode {
    //@Override
    public void initialize() {
        GamepadEx gamepadDriver = new GamepadEx(gamepad1);
        GamepadEx gamepadsystem = new GamepadEx(gamepad2);

        SampleTankDrive sampleTankDrive = new SampleTankDrive(hardwareMap);
        TankSubsystem driveSystem = new TankSubsystem(sampleTankDrive);
        BrazoHD brazoHD= new BrazoHD(telemetry,hardwareMap);
        Pinza pinza = new Pinza(telemetry, hardwareMap);
        //Escalador escalador = new Escalador(hardwareMap,telemetry);
       // Roller roller = new Roller(telemetry,hardwareMap);

        driveSystem.setDefaultCommand(new TankDriveCommand(
                driveSystem, () -> -gamepadDriver.getLeftY(), gamepadDriver::getRightX
        ));

        schedule(new RunCommand(() -> {
            driveSystem.update();
            telemetry.addData("Heading", driveSystem.getPoseEstimate().getHeading());
            telemetry.update();
        }));



       new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(pinza::close));
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(pinza::open));







      /* new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.Y)
                .whileHeld(()-> brazo.poder(-.6))
                .whenReleased(()->brazo.poder(0));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.A)
                .whileHeld(()-> brazo.poder(.6))
                .whenReleased(()->brazo.poder(0));

       */








       // new GamepadButton(new GamepadEx(gamepad2),GamepadKeys.Button.B)
         //       .toggleWhenPressed(()-> pinza.up(),()-> pinza.down());






       new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.Y)
                .whenPressed(() -> brazoHD.posiciones(4770, .5));


        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.B)
                .whenPressed(() -> brazoHD.posiciones(4575, .5));


        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.A)
                .whenPressed(() -> brazoHD.posiciones(4780, .5));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.X)
               .whenPressed(()-> brazoHD.posiciones(4350,.5));

        new GamepadButton(new GamepadEx(gamepad2),GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()-> brazoHD.posiciones(1550,.5));




        gamepadsystem.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> brazoHD.posiciones(brazoHD.getArmPose()-400,0.5))
                .whenReleased(() -> brazoHD.posiciones(brazoHD.getArmPose(),0));


        gamepadsystem.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> brazoHD.posiciones(brazoHD.getArmPose()+400,0.5))
                .whenReleased(() -> brazoHD.posiciones(brazoHD.getArmPose(),0));

        //SMALLER MANUAL
        gamepadsystem.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> brazoHD.posiciones(brazoHD.getArmPose()+50,0.5));

        gamepadsystem.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> brazoHD.posiciones(brazoHD.getArmPose()-50,0.5));





       // gamepadsystem.getGamepadButton(GamepadKeys.Button)





        /*new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.Y)
                .whenPressed(()->escalador.setPower(.8));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.A)
                .whenPressed(()->escalador.setPower(-.8));




        /*new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(roller::setRoller))
                .whenReleased(new InstantCommand(roller::stop));


        new GamepadButton(new GamepadEx(gamepad1),GamepadKeys.Button.LEFT_BUMPER)
              .whenPressed(new InstantCommand(roller::out))
            .whenReleased(new Instant
            Command(roller::stop));
         */








       //roller.setDefaultCommand(new RollerCommand(roller,gamepadDriver));


        gamepadDriver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(driveSystem::toggleInverted));


        gamepadDriver.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(driveSystem::bajarVel));




    }


}
