package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Initialization.Variables;

@TeleOp(name = "Sensor Telemetry", group = "Telemetry")
public class SensorsTelemetry extends OpMode {

    Variables var;
    Motors motors;
    RobotHardwareMap robot;

    @Override
    public void init() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;

        robot = var.robot;
    }

    @Override
    public void loop() {
        telemetry.addData("IMU", var.sensors.imuAngle().firstAngle);
        telemetry.update();
    }
}
