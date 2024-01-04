package org.firstinspires.ftc.teamcode.old.based.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Deprecated
public class BulkReader extends SubsystemBase {
    List<LynxModule> modules;

    public BulkReader(HardwareMap hardwareMap){
        super();
        modules = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : modules){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    @Override
    public void periodic(){
        for (LynxModule hub: modules){
            hub.clearBulkCache();
        }
    }
}
