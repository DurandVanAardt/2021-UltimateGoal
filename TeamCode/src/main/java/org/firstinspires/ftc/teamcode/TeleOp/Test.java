package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;


@TeleOp(name = "Test", group = "TeleOp")
public class Test extends OpMode {

    Variables var;
    Motors motors;
    RobotHardwareMap robot;

    DriveTrain driveTrain = DriveTrain.STOP;
    Shooter shooter = Shooter.REST;

    private boolean turnFirst = true;
    private boolean begin;
    private boolean turning = false;

    @Override
    public void init() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;

        robot = var.robot;

    }

    @Override
    public void loop() {

//        if (gamepad1.b) {
//            // right
//            motors.strafe(135 * Math.PI / 180, 1, true);
//        }else if (gamepad1.y) {
//            // forward
//            motors.strafe(45 * Math.PI / 180, 1, true);
//        }else if (gamepad1.x) {
//            // left
//            motors.strafe(-45 * Math.PI / 180, 1, true);
//        }else if (gamepad1.a) {
//            // reverse
//            motors.strafe(-135 * Math.PI / 180, 1, true);
//        }else {
//            motors.strafe(0, 0, false);
//
//        }

//        if (gamepad1.right_trigger != 0)
//            driveTrain = DriveTrain.STRAFER;
//        else
//        if (gamepad1.left_trigger != 0)
//            driveTrain = DriveTrain.STRAFEL;
//        else
//        if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0)
//            driveTrain = DriveTrain.DRIVE;
//        else
//            driveTrain = DriveTrain.STOP;


//        motors.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        robot.leftFront.setPower(1);
        robot.leftBack.setPower(1);
        robot.rightFront.setPower(1);
        robot.rightBack.setPower(1);

//        if (turning || gamepad1.a) {
//            if (turnFirst) {
//                var.resetAngle();
//                turnFirst = false;
//                motors.pidRotate.reset();
//                motors.pidRotate.enable();
//                turning = motors.rotate(90);
//            }
//            turning = motors.rotate(90);
//        }

//        stateMachine(driveTrain, shooter);


//        double distanceL = var.robot.distanceL.getDistance(DistanceUnit.MM);
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

    private void stateMachine(DriveTrain driveTrain, Shooter shooter) {
        driveTrainState(driveTrain);
        shooterState(shooter);
    }

    private void driveTrainState(DriveTrain driveTrain) {

        switch (driveTrain) {

            case STRAFEL:
                motors.strafe(-90, gamepad1.left_trigger, begin);

                if (begin)
                    begin = false;

                break;

            case STRAFER:
                motors.strafe(90, gamepad1.right_trigger, begin);

                if (begin)
                    begin = false;

                break;

            case DRIVE:
                motors.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

                if (begin)
                    begin = false;

                break;

            case STOP:
                motors.strafe(0,0,true);

                begin = true;

                break;
        }
    }

    private void shooterState(Shooter shooter) {

        switch (shooter) {

            case FIRE:
                break;

            case ADJUSTANGLE:
                break;

            case PICKUP:
                break;

            case REST:
                break;
        }
    }
}
