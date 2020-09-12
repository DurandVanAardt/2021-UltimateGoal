package org.firstinspires.ftc.teamcode.Resources;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Initialization.Variables;

import java.util.Locale;


//@TeleOp(name = "Sensors", group = "TeleOp")
public class Sensors extends Thread {
    private Variables var;

    private int count = 0;
    private double distanceLAvg;
    private double distanceRAvg;
    private double distanceBAvg;
    private double distanceFMAvg;
    private Orientation lastAngles;
//    public BNO055IMU imu;

    public Sensors(Variables var){

        this.var = var;
        var.setOpModeActive(true);

    }

    @Override
    public void start(){

        //Start the Actual Thread
        run();

    }

    @Override
    public void run(){

        while (var.isOpModeActive()){
            imu();
            distanceSensors();
            colourSensor();
            encoders();
//            setVar();
//            var.count ++;
        }
    }

    private void imu() {
        Orientation angles = var.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        var.setHeading(formatAngle(angles.angleUnit, angles.firstAngle));
        var.setRoll(formatAngle(angles.angleUnit, angles.secondAngle));
        var.setPitch(formatAngle(angles.angleUnit, angles.thirdAngle));

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        var.setGlobalAngle(var.getGlobalAngle() + deltaAngle);

        lastAngles = angles;

    }

    private void encoders() {
        var.setAvgEncoder((var.robot.leftFront.getCurrentPosition()+
                var.robot.rightFront.getCurrentPosition()+
                var.robot.leftBack.getCurrentPosition()+
                var.robot.rightBack.getCurrentPosition()) / 4);
    }

    private void colourSensor() {

    }

    private void distanceSensors(){
        double distanceLTemp = var.robot.distanceL.getDistance(DistanceUnit.MM);
        double distanceRTemp = var.robot.distanceR.getDistance(DistanceUnit.MM);
        double distanceBTemp = var.robot.distanceB.getDistance(DistanceUnit.MM);
        double distanceFMTemp = var.robot.distanceFM.getDistance(DistanceUnit.MM);
        count ++;
        if (count != 5){
            distanceLAvg += distanceLTemp;
            distanceRAvg += distanceRTemp;
            distanceBAvg += distanceBTemp;
            distanceFMAvg += distanceFMTemp;
        }else {
            distanceLAvg /= 5;
            distanceRAvg /= 5;
            distanceBAvg /= 5;
            distanceFMAvg /= 5;
            var.setDistanceL(distanceLAvg);
            var.setDistanceR(distanceRAvg);
            var.setDistanceB(distanceBAvg);
            var.setDistanceFM(distanceFMAvg);

            count = 0;
        }
    }

    public double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.parseDouble(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public Variables getVar() {
        return var;
    }

    public void resetAngle() {
        lastAngles = var.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        var.setGlobalAngle(0);
    }
//
//    public RobotHardwareMap getRobot() {
//        return robot;
//    }
//
//    public void setRobot(RobotHardwareMap robot) {
//        this.robot = robot;
//    }
}