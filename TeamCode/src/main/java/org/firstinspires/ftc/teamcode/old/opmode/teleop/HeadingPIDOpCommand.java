package org.firstinspires.ftc.teamcode.old.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.old.based.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.common.Constants;

@TeleOp
public class HeadingPIDOpCommand extends CommandOpMode {

    Drivetrain m_drive;
    GamepadEx m_commandGamepad;

    @Override
    public void initialize(){
        this.m_commandGamepad = new GamepadEx(gamepad1);
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        m_drive = new Drivetrain(0, this.hardwareMap, this.telemetry);


        m_commandGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(
                        m_drive::resetYaw,
                        m_drive
                )
        );

        m_commandGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(
                new InstantCommand(
                        () -> m_drive.setDefaultCommand(
                                new RunCommand(
                                        () -> m_drive.updateExperimental(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x),
                                        m_drive
                                )
                        )
                ),
                new InstantCommand(
                        () -> m_drive.setDefaultCommand(
                                new RunCommand(
                                        () -> m_drive.drive(-gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.left_stick_y,0, Constants.Drive.DRIVE_POWER_MODIFIER),
                                        m_drive
                                )
                        )
                )
        );

        m_drive.setDefaultCommand(
                new RunCommand(
                        () -> m_drive.updateExperimental(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x),
                        m_drive
                )
        );

    }

    @Override
    public void run(){
        super.run();
        updateTelemetry(telemetry);
    }
}
