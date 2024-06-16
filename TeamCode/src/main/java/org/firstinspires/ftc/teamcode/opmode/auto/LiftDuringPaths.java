package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.based.command.FollowPathCommand;
import org.firstinspires.ftc.teamcode.based.subsystem.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.based.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.util.LoopTimer;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Test")
@Config
public class LiftDuringPaths extends LinearOpMode {
    private LiftSubsystem m_lift;
    private AutoDrivetrain drive;

    private LoopTimer loopTimer;
    private BulkReader reader;

    private TrajectorySequence trajectory;

    public static int TIME_WAIT = 500;

    @Override
    public void runOpMode(){
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new AutoDrivetrain(hardwareMap);
        m_lift = new LiftSubsystem(hardwareMap, telemetry);
        reader = new BulkReader(hardwareMap);

        loopTimer = new LoopTimer(telemetry);

        trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(20)
                .build();

        waitForStart();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_lift.setTarget(Constants.ConstantsLift.MAX), m_lift),
                        new WaitCommand(TIME_WAIT),
                        new FollowPathCommand(drive, trajectory)
                )
        );

        while (opModeIsActive() && !isStopRequested()){
            CommandScheduler.getInstance().run();
            loopTimer.updateLoop();
            telemetry.update();
        }
        loopTimer.endLoop();
    }
}
