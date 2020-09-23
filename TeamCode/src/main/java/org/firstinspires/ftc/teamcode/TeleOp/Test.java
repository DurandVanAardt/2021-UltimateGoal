package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;

@TeleOp(name = "Test", group = "TeleOp")
public class Test extends OpMode {

    Variables var;
    Motors motors;
    RobotHardwareMap robot;
    private double hingeHeight;
    private double towerHeight;
    private double cameraDistance;
    private double shooterAngle;

    @Override
    public void init() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;
        robot = var.robot;
    }

    boolean xy = true;
    @Override
    public void loop() {

//        if (gamepad1.x || gamepad1.y) {
//            xy = gamepad1.x;
//        }
//
//        if (xy) {
//            motors.driveStrafe(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,1);
//        }
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

        if (gamepad1.b) {
            motors.driveStrafe(135 * Math.PI / 180, 1);
        }else
        if (gamepad1.y) {
            motors.driveStrafe(45 * Math.PI / 180, 1);
        }else
        if (gamepad1.x) {
            motors.driveStrafe(-45 * Math.PI / 180, 1);
        }else
        if (gamepad1.a) {
            motors.driveStrafe(-135 * Math.PI / 180, 1);
        }else {
            motors.stop();
        }

        telemetry.addData("hi", (Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4) * 180 / Math.PI);
        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", gamepad1.left_stick_y);


//        double distanceL = var.robot.distanceL.getDistance(DistanceUnit.MM);
//
//
//        hingeHeight = 100; //die hoogte vd shooter (vanaf camera)
//
//        // onthou om die distance te minus vanaf camera tot shooter (cameraDistance)
//
//        towerHeight = 800 - hingeHeight; //mm
//
//        shooterAngle = (Math.atan2(towerHeight, cameraDistance)) /180;
//
//        robot.shooterAngleServo.setPosition(shooterAngle);

    }

    @Override
    public void stop() {
    }
}
