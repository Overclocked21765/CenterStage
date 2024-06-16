package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.based.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.util.LoopTimer;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

@Autonomous(group = "Test")
public class LiftDuringPathsNoCommands extends OpMode {
    private SampleMecanumDrive drive;
    private LiftSubsystem lift;
    private BulkReader reader;

    private Trajectory trajectory;

    private LoopTimer loopTimer;
    private TestStates state;

    private ElapsedTime liftTimer;

    public enum TestStates{
        SLIDE_MOVING,
        ROBOT_MOVING,
        STILL
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        reader = new BulkReader(hardwareMap);
        lift = new LiftSubsystem(hardwareMap, telemetry);

        loopTimer = new LoopTimer(telemetry);
        liftTimer = new ElapsedTime();

        trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();


        state = TestStates.SLIDE_MOVING;
    }

    @Override
    public void init_loop(){
        lift.periodic();
    }

    @Override
    public void start(){
        loopTimer.reset();
        lift.setTarget(Constants.ConstantsLift.MAX);
        liftTimer.reset();
    }

    @Override
    public void loop() {
        switch (state){
            case SLIDE_MOVING:
                if (liftTimer.time() > 0.5){
                    drive.followTrajectoryAsync(trajectory);
                    state = TestStates.ROBOT_MOVING;
                }
                break;
            case ROBOT_MOVING:
                if (!drive.isBusy()){
                    state = TestStates.STILL;
                }
                break;
        }
        telemetry.addData("State: ", state);
        lift.periodic();
        drive.update();
        loopTimer.updateLoop();
        reader.periodic();
    }
}
