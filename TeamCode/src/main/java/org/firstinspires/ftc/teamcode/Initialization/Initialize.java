package org.firstinspires.ftc.teamcode.Initialization;

import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.Initialization.Variables;

public class Initialize {

    public Initialize() {

    }

    public Variables Init(HardwareMap hardwareMap) {
        Variables var = new Variables(hardwareMap);
        var.init(var);

        Thread Tsensors = new Thread(var.sensors);
        Tsensors.start();
        return var;
    }
}
