package org.firstinspires.ftc.teamcode.old.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.based.subsystem.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.old.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.old.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class DepAutoRedRight extends OpMode {
    SampleMecanumDrive drive;

    EndEffectorSubsystem effector;

    boolean stopped;

    TrajectorySequence traj1;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        effector = new EndEffectorSubsystem(hardwareMap, telemetry);
        traj1 = drive.trajectorySequenceBuilder(new Pose2d(11.76, -59.58, Math.toRadians(90.00)))
                .splineTo(new Vector2d(12.14, -11.20), Math.toRadians(89.55))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(58, -11.20))
                .build();

        stopped = false;

        effector.grab();

        drive.setPoseEstimate(traj1.start());
    }

    @Override
    public void init_loop(){
        telemetry.addLine("Check auto name");
    }

    @Override
    public void start(){
        drive.followTrajectorySequenceAsync(traj1);
    }

    @Override
    public void loop() {
        if (!stopped){
            effector.grab();
            if (!drive.isBusy()){
                effector.release();
                stopped = true;
            }
        }
        drive.update();
        telemetry.addData("h", stopped);
    }
}
