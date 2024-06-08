package org.firstinspires.ftc.teamcode.TeleOPs;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.Commands.ArmCommand;
import org.firstinspires.ftc.teamcode.Commands.TankDriveCommand;


import org.firstinspires.ftc.teamcode.Subsystems.BrazoCore;
import org.firstinspires.ftc.teamcode.Subsystems.BrazoHD;
import org.firstinspires.ftc.teamcode.Subsystems.Pinza;
import org.firstinspires.ftc.teamcode.Subsystems.TankSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@TeleOp
public class TeleOpCore extends CommandOpMode {
    @Override
    public void initialize() {
        SampleTankDrive sampleTankDrive = new SampleTankDrive(hardwareMap);
        TankSubsystem driveSystem = new TankSubsystem(sampleTankDrive);
        Pinza pinza = new Pinza(telemetry, hardwareMap);
        BrazoCore brazoCore = new BrazoCore(telemetry,hardwareMap);

        GamepadEx chassisDriver = new GamepadEx(gamepad1);
        GamepadEx subsystemsDriver = new GamepadEx(gamepad2);

        //Tank----------------------------------

        driveSystem.setDefaultCommand(new TankDriveCommand(driveSystem, chassisDriver::getLeftY
                ,chassisDriver::getRightX));

        chassisDriver.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(driveSystem::bajarVel));

         /*new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_UP)
                .whileHeld(()-> brazoCore.poder(-.7))
                .whenReleased(()->brazoCore.poder(0));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(()-> brazoCore.poder(.7))
                .whenReleased(()->brazoCore.poder(0));

        new GamepadButton(new GamepadEx(gamepad2),GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(()-> brazoCore.poder(-.3))
                .whenReleased(()->brazoCore.poder(0));


*/


        //Arm------------------------------------

        subsystemsDriver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> brazoCore.setPosition(0,0.5));

        subsystemsDriver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> brazoCore.setPosition(50,0.5));

        subsystemsDriver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> brazoCore.setPosition(350,0.3));

        subsystemsDriver.getGamepadButton(GamepadKeys.Button.X)
                        .whenPressed(brazoCore::resetArmTicks);

        //MANUAL
        subsystemsDriver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> brazoCore.setPosition(brazoCore.getArm1Pose()-3000,.7))
                .whenReleased(() -> brazoCore.setPosition(brazoCore.getArm1Pose(),0));


        subsystemsDriver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> brazoCore.setPosition(brazoCore.getArm1Pose()+3000,.7))
                .whenReleased(() -> brazoCore.setPosition(brazoCore.getArm1Pose(),0));





        subsystemsDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> brazoCore.setPosition(brazoCore.getArm1Pose()-15,.7));
        subsystemsDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> brazoCore.setPosition(brazoCore.getArm1Pose()-15,.7));



        //Servo -------------------------------------------------------------

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(pinza::open);

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(pinza::close);

        //-------------------------------------------------------------------------------

        schedule(new RunCommand(()-> {
            driveSystem.update();
            telemetry.addData("Heading", driveSystem.getPoseEstimate().getHeading());
            telemetry.update();
        }));

    }



}

