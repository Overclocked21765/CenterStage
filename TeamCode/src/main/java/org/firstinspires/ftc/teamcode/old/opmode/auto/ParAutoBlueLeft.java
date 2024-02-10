package org.firstinspires.ftc.teamcode.old.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.old.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.old.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class ParAutoBlueLeft extends OpMode {
    TrajectorySequence trajectory;
    SampleMecanumDrive drive;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        trajectory =  drive.trajectorySequenceBuilder(new Pose2d(11.95, 62.78, Math.toRadians(-85.39)))
                .splineTo(new Vector2d(11.76, 34.73), Math.toRadians(269.62))
                .splineTo(new Vector2d(11.20, 15.53), Math.toRadians(268.32))
                .splineTo(new Vector2d(36.80, 13.08), Math.toRadians(-5.46))
                .splineTo(new Vector2d(58.45, 13.46), Math.toRadians(1.00))
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
