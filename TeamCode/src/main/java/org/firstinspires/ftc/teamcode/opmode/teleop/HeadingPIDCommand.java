package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.based.command.DriveCommand;
import org.firstinspires.ftc.teamcode.based.subsystem.Drivetrain;

@Config
@TeleOp
public class HeadingPIDCommand extends CommandOpMode {
    Drivetrain m_drive;
    GamepadEx m_gamepad;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        m_gamepad = new GamepadEx(gamepad1);
        m_drive = new Drivetrain(0, hardwareMap, telemetry, m_gamepad);

        m_gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> m_drive.resetYaw(), m_drive)
        );

        m_drive.setDefaultCommand(
                new DriveCommand(m_drive)
        );

        schedule(new RunCommand(telemetry::update));
    }
}
