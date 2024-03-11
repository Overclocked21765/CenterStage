package org.firstinspires.ftc.teamcode.based.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.based.subsystem.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class FollowPathCommand extends CommandBase {
    private final TrajectorySequence m_sequence;
    private final AutoDrivetrain m_drive;

    public FollowPathCommand(AutoDrivetrain drive, TrajectorySequence sequence){
        this.m_drive = drive;
        this.m_sequence = sequence;

        this.addRequirements(drive);
    }

    @Override
    public void initialize(){
        m_drive.followTrajectorySequenceAsync(m_sequence);
    }

    @Override
    public void execute(){
        m_drive.update();
    }

    @Override
    public boolean isFinished(){
        return !m_drive.isBusy();
    }
}
