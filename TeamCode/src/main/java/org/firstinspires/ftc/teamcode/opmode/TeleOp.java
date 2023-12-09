package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.based.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.based.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    public GamepadEx gamepad;
    public Drivetrain m_drive;
    public LiftSubsystem m_lift;
    public ClawSubsystem m_claw;
    public BulkReader m_bulkReader;


    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);

        m_drive = new Drivetrain(hardwareMap);
        m_lift = new LiftSubsystem(hardwareMap);
        m_claw = new ClawSubsystem(hardwareMap);
        m_bulkReader = new BulkReader(hardwareMap);

        Trigger rightTrigger = new Trigger(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1
        );

        Trigger leftTrigger = new Trigger(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1
        );



        rightTrigger.whenActive(
                new RunCommand(
                        () -> m_lift.move(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
                )
        ).whenInactive(
                new InstantCommand(
                        () -> m_lift.move(0)
                )
        );

        leftTrigger.whenActive(
                new RunCommand(
                        () -> m_lift.move(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
                )
        ).whenInactive(
                new InstantCommand(
                        () -> m_lift.move(0)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                        .toggleWhenPressed(
                                new InstantCommand(() -> m_claw.grab()),
                                new InstantCommand(() -> m_claw.release())
                        );





        m_drive.setDefaultCommand(
                new RunCommand(
                        () -> m_drive.drive(gamepad.getLeftX(), -gamepad.getLeftY(), gamepad.getRightX())
                )
        );
    }
}
