package org.firstinspires.ftc.teamcode.Initialization;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Initialize {

    public Initialize() {

    }

    public Variables Init(HardwareMap hardwareMap) {
        Variables var = new Variables(hardwareMap);
        var.init(var);

        return var;
    }
}
