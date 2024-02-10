package org.firstinspires.ftc.teamcode.old.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.old.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.old.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class ParkAutoBlueRight extends OpMode {
    TrajectorySequence trajectory;
    SampleMecanumDrive drive;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        trajectory = drive.trajectorySequenceBuilder(new Pose2d(-36.24, 63.72, Math.toRadians(268.92)))
                .splineTo(new Vector2d(-36.80, 37.18), Math.toRadians(268.78))
                .splineTo(new Vector2d(-37.55, 17.04), Math.toRadians(267.86))
                .splineTo(new Vector2d(-13.65, 11.58), Math.toRadians(-12.86))
                .splineTo(new Vector2d(16.47, 11.95), Math.toRadians(0.72))
                .splineTo(new Vector2d(61.27, 12.71), Math.toRadians(0.96))
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
