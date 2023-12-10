package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.based.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.based.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.based.subsystem.HeadingPID;
import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    public GamepadEx gamepad;
    public HeadingPID m_drive;
    public LiftSubsystem m_lift;
    public ClawSubsystem m_claw;
    public BulkReader m_bulkReader;

    boolean buttonAlreadyPressed;

    boolean experimental;


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad = new GamepadEx(gamepad1);

        m_drive = new HeadingPID(0);
        m_drive.init(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT, telemetry);
        buttonAlreadyPressed = false;

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

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(
                new InstantCommand(() -> experimental = true),
                new InstantCommand(() -> experimental = false)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new FunctionalCommand(
                        () -> m_lift.resetting = true,
                        () -> m_lift.resetMovement(),
                        (interrupted) -> {m_lift.reset(); m_lift.resetting = false;},
                        () -> gamepad.getButton(GamepadKeys.Button.START),
                        m_lift

                )
        );





        experimental = false;
        telemetry.addLine("Initialization complete");
        telemetry.update();
    }

    @Override
    public void run(){
        super.run();
        telemetry.addData("Left y: ", gamepad.getLeftY());

        if (experimental){
            m_drive.updateExperimental(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        } else {
            m_drive.update(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }
        //double leftX, double leftY, double rightX
        if (gamepad1.y && !buttonAlreadyPressed){
            m_drive.resetYaw();
        }
        buttonAlreadyPressed = gamepad1.y;
        telemetry.addData("experimental: ", experimental);
        updateTelemetry(this.telemetry);


    }


}
