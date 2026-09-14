package org.firstinspires.ftc.teamcode.subsystems.hardware.imu;

import com.seattlesolvers.solverslib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.constants.IMUConstants;

public class IMUSubsystem {
    private final RevIMU imu;

    public IMUSubsystem(HardwareMap hardwareMap) {
        imu = new RevIMU(hardwareMap, IMUConstants.IMUName);
        imu.init();
    }

    public double getHeading() { 
        return imu.getHeading();
    }
}
