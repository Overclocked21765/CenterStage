package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.based.subsystem.ArmSubsystem;

@TeleOp(group = "Hardware")
@Config
public class HardwareArm extends OpMode {
    private ArmSubsystem m_lift;

    public static double target = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        m_lift = new ArmSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        m_lift.setPositions(target, target);
    }
}
