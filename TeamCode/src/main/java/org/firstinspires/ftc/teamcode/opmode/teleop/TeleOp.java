package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.based.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.based.subsystem.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.old.based.subsystem.HeadingPID;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    private LiftSubsystem m_lift;
    private ArmSubsystem m_arm;
    private EndEffectorSubsystem m_effector;
    private BulkReader m_reader;
    private HeadingPID drive;

    private GamepadEx m_driveController;

    private boolean yPressed;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);

        m_driveController = new GamepadEx(gamepad1);

        m_lift = new LiftSubsystem(hardwareMap);
        m_arm = new ArmSubsystem(hardwareMap);
        m_effector = new EndEffectorSubsystem(hardwareMap);
        m_reader = new BulkReader(hardwareMap);

        yPressed = false;

        drive = new HeadingPID(0);
        drive.init(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT, this.telemetry);

        m_driveController.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                new StartEndCommand(
                        () -> m_effector.grab(),
                        () -> {if (m_arm.getState() == ArmSubsystem.ArmStates.INTAKE) m_effector.stow();},
                        m_effector,
                        m_arm
                ).withTimeout(500),
                new InstantCommand(
                        () -> m_effector.release(),
                        m_effector
                )
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_arm.setDepositPosition(), m_arm),
                        new WaitCommand(200),
                        new InstantCommand(() -> m_effector.passThrough(), m_effector),
                        new WaitCommand(1000),
                        new InstantCommand(() -> m_effector.moveToDeposit(), m_effector)
                )
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_effector.passThrough(), m_effector),
                        new WaitCommand(300),
                        new InstantCommand(() -> m_arm.setIntakePosition(), m_arm),
                        new WaitCommand(500),
                        new InstantCommand(() -> m_effector.moveToIntake(), m_effector),
                        new InstantCommand(() -> m_effector.release(), m_effector)
                )
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(
                new InstantCommand(() -> m_effector.grabLeft(), m_effector),
                new InstantCommand(() -> m_effector.releaseLeft(), m_effector)
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(
                new InstantCommand(() -> m_effector.grabRight(), m_effector),
                new InstantCommand(() -> m_effector.releaseRight(), m_effector)
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.START).whileHeld(
                new RunCommand(() -> m_lift.doResetMovement(), m_lift)
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.START).whenReleased(
                new InstantCommand(() -> m_lift.reset())
        );

        Trigger rightTrigger = new Trigger(
                () -> m_driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1
        );

        Trigger leftTrigger = new Trigger(
                () -> m_driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1
        );



        rightTrigger.whenActive(
                new RunCommand(
                        () -> m_lift.move(m_driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_driveController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
                )
        ).whenInactive(
                new InstantCommand(
                        () -> m_lift.move(0)
                )
        );

        leftTrigger.whenActive(
                new RunCommand(
                        () -> m_lift.move(m_driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_driveController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
                )
        ).whenInactive(
                new InstantCommand(
                        () -> m_lift.move(0)
                )
        );



        /**
         * slides on triggers
         * bumpers lr claw
         * a -> grab stow
         *
         */
    }

    @Override
    public void run(){
        super.run();

        drive.updateExperimental(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        if (gamepad1.y && !yPressed){
            drive.resetYaw();
        }
        yPressed = gamepad1.y;
    }
}
