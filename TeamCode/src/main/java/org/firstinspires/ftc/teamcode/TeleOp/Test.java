package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Resources.Motors;

@TeleOp(name = "Test", group = "TeleOp")
public class Test extends OpMode {

    Variables var;
    Motors motors;

    @Override
    public void init() {
        var = new Initialize().Init(hardwareMap);
//        motors = var.motors;
    }

    boolean xy = false;
    @Override
    public void loop() {
//
//        if (gamepad1.x || gamepad1.y) {
//            xy = gamepad1.x;
//        }
//
//
//        if (xy) {
//            motors.driveStrafe(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,1);
//        }
//
//
//        else {
//
//            if (gamepad1.right_trigger != 0) {
//                motors.strafeR(gamepad1.right_trigger, 1);
//            } else if (gamepad1.left_trigger != 0) {
//                motors.strafeL(gamepad1.left_trigger, 1);
//            } else
//                motors.stop();
//
//        }

    }

    @Override
    public void stop() {
    }
}
