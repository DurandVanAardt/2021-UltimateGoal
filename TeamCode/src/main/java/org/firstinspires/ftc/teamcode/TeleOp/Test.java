package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;


@TeleOp(name = "Test", group = "TeleOp")
@Disabled
public class Test extends OpMode {

    Variables var;
    Motors motors;
    RobotHardwareMap robot;

    DriveTrain driveTrain = DriveTrain.STOP;
    Shooter shooter = Shooter.REST;

    private boolean turnFirst = true;
    private boolean begin;
    private boolean turning = false;
    private boolean turningUp;
    private boolean turningDown;
    private boolean turningLeft;
    private boolean turningRight;

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
//            motors.driveStrafe(135 * Math.PI / 180, 1, true);
//        }else if (gamepad1.y) {
//            // forward
//            motors.driveStrafe(45 * Math.PI / 180, 1, true);
//        }else if (gamepad1.x) {
//            // left
//            motors.driveStrafe(-45 * Math.PI / 180, 1, true);
//        }else if (gamepad1.a) {
//            // reverse
//            motors.driveStrafe(-135 * Math.PI / 180, 1, true);
//        }else {
//            motors.driveStrafe(0, 0, false);
//        }


        telemetry.addData("hi", motors.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
//
//        else if (gamepad1.dpad_up || turningUp)
//            driveTrain = DriveTrain.TURNUP;
//
//        else if (gamepad1.dpad_down || turningDown)
//            driveTrain = DriveTrain.TURNDOWN;
//
//        else if (gamepad1.dpad_left || turningLeft)
//            driveTrain = DriveTrain.TURNLEFT;
//
//        else if (gamepad1.dpad_right || turningRight)
//            driveTrain = DriveTrain.TURNRIGHT;
//
        if (gamepad1.right_trigger != 0)
            driveTrain = DriveTrain.STRAFER;
        else
        if (gamepad1.left_trigger != 0)
            driveTrain = DriveTrain.STRAFEL;
        else
        if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0)
            driveTrain = DriveTrain.DRIVE;
        else
            driveTrain = DriveTrain.STOP;

//        stateMachine(driveTrain, shooter);

//        telemetry.addData("State", driveTrain);
//
        stateMachine(driveTrain, shooter);

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
                 telemetry.addData("Angle", motors.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
//
//                if (begin)
//                    begin = false;

                break;

            case TURNUP:

                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                    turningUp = motors.rotate(0);
                }

                turningUp = motors.rotate(0);


                turnFirst = !turningUp;

                break;

            case TURNDOWN:

                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                    turningDown = motors.rotate(-180);
                }

                turningDown = motors.rotate(180);

                turnFirst = !turningDown;

                break;

            case TURNLEFT:

                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                    turningLeft = motors.rotate(90);
                }

                turningLeft = motors.rotate(90);

                turnFirst = !turningLeft;

                break;

            case TURNRIGHT:

                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                    turningRight = motors.rotate(-90);
                }

                turningRight = motors.rotate(-90);

                turnFirst = !turningRight;

                break;

            case STOP:
                motors.stop();

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

            case SUCKER:
                break;

            case REST:
                break;
        }
    }
}
