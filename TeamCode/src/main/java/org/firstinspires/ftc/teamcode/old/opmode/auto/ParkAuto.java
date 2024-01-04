package org.firstinspires.ftc.teamcode.old.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.old.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.old.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class ParkAuto extends OpMode {
    TrajectorySequence trajectory;
    SampleMecanumDrive drive;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        trajectory = drive.trajectorySequenceBuilder(new Pose2d(11.58, -64.85, Math.toRadians(90.00)))
                .splineTo(new Vector2d(10.07, -39.06), Math.toRadians(90.00))
                .splineTo(new Vector2d(12.33, -17.60), Math.toRadians(83.99))
                .splineTo(new Vector2d(35.29, -15.15), Math.toRadians(6.08))
                .splineTo(new Vector2d(55.62, -15.53), Math.toRadians(-1.06))
                .build();
        drive.setPoseEstimate(trajectory.start());
    }

    @Override
    public void start(){
        drive.followTrajectorySequenceAsync(trajectory);
    }

    @Override
    public void loop() {
        drive.update();
    }
}
