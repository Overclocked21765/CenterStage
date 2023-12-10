package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class ParkAutoRedLeft extends OpMode {
    TrajectorySequence trajectory;
    SampleMecanumDrive drive;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        trajectory = drive.trajectorySequenceBuilder(new Pose2d(-37.74, -61.08, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-38.49, -39.44), Math.toRadians(91.99))
                .splineTo(new Vector2d(-38.12, -7.62), Math.toRadians(89.32))
                .splineTo(new Vector2d(-15.15, -8.00), Math.toRadians(-0.94))
                .splineTo(new Vector2d(17.98, -10.07), Math.toRadians(-3.58))
                .splineTo(new Vector2d(60.89, -10.07), Math.toRadians(0.00))
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
