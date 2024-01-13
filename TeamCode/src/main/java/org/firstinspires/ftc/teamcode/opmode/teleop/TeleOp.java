package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.based.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.based.subsystem.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.old.based.subsystem.HeadingPID;

public class TeleOp extends CommandOpMode {
    private LiftSubsystem lift;
    private ArmSubsystem arm;
    private EndEffectorSubsystem claw;
    private BulkReader reader;
    private HeadingPID drive;

    private GamepadEx m_driveController;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);

        lift = new LiftSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        claw = new EndEffectorSubsystem(hardwareMap);
        reader = new BulkReader(hardwareMap);

        drive = new HeadingPID(0);
        drive.init(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT, this.telemetry);
    }
}
