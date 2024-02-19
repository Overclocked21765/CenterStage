package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.based.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.based.subsystem.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.based.subsystem.HeadingPID;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    private LiftSubsystem m_lift;
    private ArmSubsystem m_arm;
    private EndEffectorSubsystem m_effector;
    private BulkReader m_reader;
    private HeadingPID drive;

    private GamepadEx m_driveController;

    private boolean yPressed;

    public static long timeReachDeposit = 1200;

    public static long timeWristStraighten = 500;
    public static long timeDepositToEffector = 1000;
    public static long timeEffectorTo02 = 200;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);

        m_driveController = new GamepadEx(gamepad1);

        m_lift = new LiftSubsystem(hardwareMap, telemetry);
        m_arm = new ArmSubsystem(hardwareMap);
        m_effector = new EndEffectorSubsystem(hardwareMap, telemetry);
        m_reader = new BulkReader(hardwareMap);

        yPressed = false;

        drive = new HeadingPID(0);
        drive.init(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT, this.telemetry);

//        m_driveController.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
//                new StartEndCommand(
//                        () -> m_effector.grab(),
//                        () -> {if (m_arm.getState() == ArmSubsystem.ArmStates.INTAKE) m_effector.stow(); m_arm.setPositions(0.25, 0.25);},
//                        m_effector,
//                        m_arm
//                ).withTimeout(500),
//                new InstantCommand(
//                        () -> m_effector.release(),
//                        m_effector
//                )
//        );

        m_driveController.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(
                        () -> this.m_effector.updateClaw(),
                        this.m_effector
                )
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new FunctionalCommand(
                                () -> m_lift.setTarget(Constants.Lift.GROUND_POSITION),
                                () -> {},
                                (interrupted) -> {},
                                () -> m_lift.getCurrentPosition() <= Constants.Lift.RED_ZONE,
                                m_lift
                        ),
                        new InstantCommand(() -> m_effector.passThrough(), m_effector),
                        new WaitCommand(timeWristStraighten),
                        new InstantCommand(() -> m_effector.grab(), m_effector),
                        new InstantCommand(() -> m_arm.setPositions(0.2, 0.2), m_arm),
                        new WaitCommand(timeDepositToEffector),
                        new InstantCommand(() -> m_effector.moveToIntake(), m_effector),
                        new WaitCommand(timeEffectorTo02),
                        new InstantCommand(() -> m_arm.setIntakePosition(), m_arm)
                )
        );

//        m_driveController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new SequentialCommandGroup(
//                        new FunctionalCommand(
//                                () -> m_lift.setTarget(Constants.Lift.GROUND_POSITION),
//                                () -> {},
//                                (interrupted) -> {},
//                                () -> m_lift.getCurrentPosition() <= Constants.Lift.RED_ZONE,
//                                m_lift
//                        ),
//                        new InstantCommand(() -> m_effector.passThrough(), m_effector),
//                        new WaitCommand(timeWristStraighten),
//                        new InstantCommand(() -> m_effector.grab(), m_effector),
//                        new InstantCommand(() -> m_arm.setIntakePosition(), m_arm),
//                        new WaitCommand(timeThroughSlidesToArmInBetween),
//                        new InstantCommand(() -> m_effector.moveToIntake(), m_effector)
//                )
//        );

        m_driveController.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_arm.setDepositPosition(), m_arm),
                        new InstantCommand(() -> m_effector.passThrough(), m_effector),
                        new WaitCommand(timeReachDeposit),
                        new InstantCommand(() -> m_effector.moveToDeposit(), m_effector)
                )
        );


        m_driveController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> this.m_effector.updateClawLeft(), m_effector)
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> this.m_effector.updateClawRight(), m_effector)
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.START).whileHeld(
                new RunCommand(() -> m_lift.doResetMovement(), m_lift)
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.START).whenReleased(
                new InstantCommand(() -> m_lift.reset())
        );

//        Trigger rightTrigger = new Trigger(
//                () -> m_driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.02
//        );
//
//        Trigger leftTrigger = new Trigger(
//                () -> m_driveController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.02
//        );

        Trigger triggers = new Trigger(
                () -> m_driveController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.02 || m_driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.02
        );



//        rightTrigger.whenActive(
//                new RunCommand(
//                        () -> m_lift.move(m_driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_driveController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)),
//                        m_lift
//                )
//        ).whenInactive(
//                new InstantCommand(
//                        () -> m_lift.move(0),
//                        m_lift
//                )
//        );
//
//        leftTrigger.whenActive(
//                new RunCommand(
//                        () -> m_lift.move(m_driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_driveController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)), m_lift
//                )
//        ).whenInactive(
//                new InstantCommand(
//                        () -> m_lift.move(0), m_lift
//                )
//        );

        triggers.whenActive(
                new InstantCommand(() -> m_lift.moveManually(m_driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_driveController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)), m_lift)
        ).whenInactive(
                new InstantCommand(() -> m_lift.stopMovement(), m_lift)
        );

        m_driveController.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_arm.setPositions(0.25, 0.25), m_arm),
                        new InstantCommand(() -> m_arm.setState(ArmSubsystem.ArmStates.NAN), m_arm)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_arm.setIntakePosition(), m_arm),
                        new InstantCommand(() -> m_arm.setState(ArmSubsystem.ArmStates.INTAKE), m_arm),
                        new InstantCommand(() -> m_effector.moveToIntake(), m_effector)
                )

        );





        /**
         * slides on triggers
         * bumpers lr claw
         * a -> grab stow
         *
         */

        drive.resetYaw();
        m_arm.setIntakePosition();

        schedule(new RunCommand(telemetry::update));
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
