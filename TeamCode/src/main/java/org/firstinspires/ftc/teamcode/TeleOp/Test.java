package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initialization.Variables;

import org.firstinspires.ftc.teamcode.Initialization.Initialize;

@TeleOp(name = "Test", group = "TeleOp")
public class Test extends OpMode {

    Variables var = new Initialize().Init(hardwareMap);

    @Override
    public void init() {

    }

    boolean xy = false;
    @Override
    public void loop() {
        if (gamepad1.x || gamepad1.y) {
            xy = gamepad1.x;
        }


        if (xy) {
            var.strafe.driveStrafe(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,1);
        }


        else {

            if (gamepad1.right_trigger != 0) {
                var.strafe.strafeR(gamepad1.right_trigger, 1, 0);
            } else if (gamepad1.left_trigger != 0) {
                var.strafe.strafeL(gamepad1.left_trigger, 1, 0);
            } else
                var.strafe.stop();
        }

    }

    @Override
    public void stop() {
    }
}
