package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;



@Autonomous(name="AutonomousBlue", group="Autonomous")
//@Disabled
public class AutonomousBlue extends LinearOpMode {

    private boolean turnFirst = true;
    private boolean turning = false;
    boolean turningRight = false;
    boolean TurnRight = false;


    Variables var;
    Motors motors;
    RobotHardwareMap robot;

    AutonomousMove DriveTrain = AutonomousMove.STOP;
    AutonomousShooter Shooter = AutonomousShooter.SHOOTERREST;

    private AutonomousMove curMoveState = AutonomousMove.STOP;

    @Override
    public void runOpMode() throws InterruptedException {


        var = new Initialize().Init(hardwareMap);
        motors = var.motors;

        robot = var.robot;



        waitForStart();

// ----Turning----//

        MoveState(DriveTrain.TURNLEFT); // Turn to 90 left

        MoveState(DriveTrain.TURNFRONT); // Turn to start position

// ----Shooter on ---- //

        motors.Fire(10000);

while (robot.shooterMotor.getCurrentPosition() < robot.shooterMotor.getTargetPosition()) {
    robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.shooterMotor.setPower(1);
}

 //----Driving forward---- //

        motors.driveStrafe(45 * Math.PI / 180, 1, true);
        while(!motors.pidStrafe.onTarget())

//
//
//        if ((robot.distanceL.getDistance(DistanceUnit.MM) < 300))
//            motors.stop();



// ----Strafe Right---- //

        while(!motors.pidStrafe.onTarget())
            motors.driveStrafe(135 * Math.PI / 180, 1, true);

//        if ((robot.distanceL.getDistance(DistanceUnit.MM) < 300))
//            motors.stop();


        telemetry.addData("IMU", var.getAngle());
        telemetry.update();


    }

    /**
     * Returns an estimation of the horizontal angle to the detected object.
     */
    void estimateAngleToObject(AngleUnit angleUnit) {
        
    }

    private void MoveState(AutonomousMove DriveTrain) {

        switch (DriveTrain) {

            case TURNLEFT:

                var.resetAngle();
                motors.pidRotate.reset();
                motors.pidRotate.enable();


                motors.rotate(90);

                while (!motors.pidRotate.onTarget())
                    motors.rotate(90);

                if (motors.pidRotate.onTarget())
                    telemetry.addData("IMU", var.getAngle());
                    sleep(2000);



                break;

            case TURNFRONT:

//                motors.pidRotate.reset();
//                motors.pidRotate.enable();


                motors.rotate(-90);

                while (!motors.pidRotate.onTarget())
                    motors.rotate(-90);

                if (motors.pidRotate.onTarget())
                    telemetry.addData("IMU", var.getAngle());
                    sleep(2000);
                break;

            case STOP:
                motors.stop();
        }

    }
        private void ShooterState(AutonomousShooter Shooter)
        {

            switch (Shooter) {
                case FIRE:

                break;
            }

        }



    }
