package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;

@TeleOp(group = "Hardware")
@Config
public class HardwareLift extends OpMode {
    private LiftSubsystem m_lift;

    public static int target = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        m_lift = new LiftSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        m_lift.setTarget(target);
        m_lift.periodic();

        telemetry.addData("Cur pos: ", m_lift.getCurrentPosition());
        telemetry.addData("Target pos: ", target);
    }
}