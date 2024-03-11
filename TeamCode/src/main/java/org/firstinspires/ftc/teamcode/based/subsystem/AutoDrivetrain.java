package org.firstinspires.ftc.teamcode.based.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

public class AutoDrivetrain extends SampleMecanumDrive implements Subsystem {
    public AutoDrivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.register();
    }
}
