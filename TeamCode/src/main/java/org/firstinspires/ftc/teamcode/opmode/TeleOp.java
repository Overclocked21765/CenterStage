package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    DriveSubsystem driveSubsystem;
    GamepadEx gamepad;


    @Override
    public void initialize(){
        gamepad = new GamepadEx(gamepad1);
        driveSubsystem = new DriveSubsystem(hardwareMap, gamepad);

        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));
    }
}
