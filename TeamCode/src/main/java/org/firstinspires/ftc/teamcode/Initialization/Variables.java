package org.firstinspires.ftc.teamcode.Initialization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.Resources.Sensors;
import org.firstinspires.ftc.teamcode.Resources.Motors;

import java.util.Locale;

@SuppressWarnings("unused")
public class Variables {
    public RobotHardwareMap robot;
    public Motors motors;
    private boolean OpModeActive = false;
    public Sensors sensors;
    private double heading;
    private double roll;
    private double pitch;
//    Orientation lastAngles;
    public double angle;
    double trueAngle;
    double avgEncoder;
    double distanceL;
    double distanceR;
    double distanceB;
    private Orientation lastAngles;
    private double globalAngle;

    public Variables(HardwareMap hardware) {
        robot = new RobotHardwareMap(hardware);
        //resetAngle();
    }

    public void init(Variables var) {
        motors = new Motors(var);
        sensors = new Sensors(var);
       resetAngle();
    }

//    public double getTrueAngle() {
//        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        trueAngle = angles.firstAngle;
//        return trueAngle;
//    }

    public double getAvgEncoder() {
        return avgEncoder;
    }

    public void setAvgEncoder(double avgEncoder) {
        this.avgEncoder = avgEncoder;
    }

    public double getDistanceL() {
        return distanceL;
    }

    public void setDistanceL(double distanceL) {
        this.distanceL = distanceL;
    }

    public double getDistanceR() {
        return distanceR;
    }

    public void setDistanceR(double distanceR) {
        this.distanceR = distanceR;
    }

    public double getDistanceB() {
        return distanceB;
    }

    public void setDistanceB(double distanceB) {
        this.distanceB = distanceB;
    }

    public double getDistanceFM() {
        return distanceFM;
    }

    public void setDistanceFM(double distanceFM) {
        this.distanceFM = distanceFM;
    }

    double distanceFM;

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getRoll() {
        return roll;
    }

    public void setRoll(double roll) {
        this.roll = roll;
    }

    public double getPitch() {
        return pitch;
    }

    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }


    public double getAngle() {
        return sensors.imu();
    }

    public void resetAngle() {
//        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        globalAngle = 0;
        sensors.resetAngle();
    }

    public double getTrueAngle() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        trueAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

        return trueAngle;
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
