package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

public class WRLRobot {
    public Blinker expansion_Hub_2;
    public Blinker expansion_Hub_3;
    public Gyroscope imu_1;
    public BNO055IMU imu;

    public DcMotor wheel1;
    public DcMotor wheel2;
    public DcMotor wheel3;
    public DcMotor wheel4;
    public DcMotor encoderY; // also for roller
    public DcMotor encoderX; // also for roller
    public DcMotorEx elbow;
    public DcMotor intake;

    public Odometry odometer;
    public Orientation angles; // used to get info from BNO055IMU

    HardwareMap hardwareMap;  // used to link code objects to real objects.
    LinearOpMode program; // the program using this module.  Robot requires access to the program to know when the program is trying to stop.

    public void init(HardwareMap hardwareMap, LinearOpMode program) {
        // SET UP IMU AS BNO055IMU:
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");
        imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        wheel1 = hardwareMap.get(DcMotor.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotor.class, "wheel2");
        wheel3 = hardwareMap.get(DcMotor.class, "wheel3");
        wheel4 = hardwareMap.get(DcMotor.class, "wheel4");
        encoderY = hardwareMap.get(DcMotor.class, "encoderY"); // also for roller
        encoderX = hardwareMap.get(DcMotor.class, "encoderX");
        elbow = hardwareMap.get(DcMotorEx.class, "Elbow");
        intake = hardwareMap.get(DcMotor.class, "intake");

        DcMotor[] motors = {wheel1, wheel2, wheel3, wheel4};
        for (int i = 0; i < 3; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        wheel4.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderX.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderY .setDirection(DcMotorSimple.Direction.REVERSE);

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometer = new Odometry(0, 0, 0);
        odometer.init(imu, encoderY, encoderX);
        this.program = program;
    }
    public void strafe(double degrees, double speed, double clicks, int millis) {
        // degrees is which direction the robot goes.
        // 0 degrees is forward, 90 is right, -90 is left and +/- 180 is backwards.
        // millis is a time limit on the method in milliseconds.
        // negative clicks will reverse the direction. please don't use negative clicks.
        // 100 clicks is about 2 inches if the robot is going forward.
        double start = (int)System.currentTimeMillis();
        // keep degrees in the correct range
        degrees += 45;
        if (degrees >= 180) {
            degrees -= 360;
        } else if (degrees < -180) {
            degrees += 360;
        }

        double theta = degrees/180*Math.PI;
        if (theta >= Math.PI) {
            theta -= Math.PI*2;
        } else if (theta < -Math.PI) {
            theta += Math.PI*2;
        }
        double x_vector = Math.cos(theta);  // calculate ratio of x distance to y distance.
        double y_vector = Math.sin(theta);


        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // the mecanum wheels work in pairs, with a pair being a wheel and the wheel across the robot.
        // thus wheels 1 and 4 are one pair, the 'y' pair, and wheels 2 and 3 are the 'x' pair.
        // due to the way trigonometric functions and mecanum wheels work, it's helpful to think of the wheel pairs as x and y components.
        int xclicks = (int)Math.round(clicks*x_vector);
        int yclicks = (int)Math.round(clicks*y_vector);
        wheel1.setTargetPosition(yclicks);
        wheel2.setTargetPosition(xclicks);
        wheel3.setTargetPosition(xclicks);
        wheel4.setTargetPosition(yclicks);

        wheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheel1.setPower(speed*y_vector); // front motors are older than the back ones and require more power.
        wheel2.setPower(speed*x_vector);
        wheel3.setPower(speed*x_vector);
        wheel4.setPower(speed*y_vector);
        // loop to make robot move and continually correct heading while moving.
        while (!this.program.isStopRequested() && (wheel1.isBusy() || wheel2.isBusy()) && (wheel3.isBusy() || wheel4.isBusy())) {
            if (start+millis < (int)System.currentTimeMillis()) {
                break;
            }
        }
        // stop.
        wheel1.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
        wheel4.setPower(0);
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void imuTurn(int degrees, double speed, int millis) {
        // has robot turn at speed power to heading degrees. degrees should be 180 >= degrees >= -180.
        // millis is a time limit on the method in milliseconds.
        double difference, sign, correct, final_speed, start = (int)System.currentTimeMillis();
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (angles.firstAngle != degrees && !this.program.isStopRequested()) {

            angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            difference = angles.firstAngle-degrees;
            if (difference < 0) { sign = -1;
            } else { sign = 1; }

            if (Math.abs(difference) > 180) {
                difference = sign * (360-Math.abs(difference));
                correct = -(difference)/60;
                if (Math.abs(correct) > 1) { correct = -sign; }
            } else {
                correct = (difference)/60;
                if (Math.abs(correct) > 1) { correct = sign; }
            }
            final_speed = speed*correct;
            if (Math.abs(final_speed) < 0.2) {
                if (final_speed > 0) {final_speed = 0.2;}
                if (final_speed < 0) {final_speed = -0.2;}
            }
            wheel2.setPower(-final_speed);
            wheel4.setPower(-final_speed);
            wheel1.setPower(final_speed);
            wheel3.setPower(final_speed);
            if (start+millis < (int)System.currentTimeMillis()) {
                break;
            }
        }
        wheel2.setPower(0);
        wheel4.setPower(0);
        wheel1.setPower(0);
        wheel3.setPower(0);
    }
    public void motorTurnNoReset(double speed, int clicks, DcMotor motor) {
        // has motor turn clicks at speed. Not very compicated.
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(clicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
    }
    public void motorTurn(double speed, int clicks, DcMotor motor) {
        // has motor turn clicks at speed. Not very compicated.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(clicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        while (motor.isBusy() && !this.program.isStopRequested()) {}
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void imuStrafe(double heading, double degrees, double speed, double clicks,
                          int adjustPower, int buffer, int millis) {
        // heading is which way the robot faces, and degrees is which direction it goes.
        // 0 degrees is forward, 90 is right, -90 is left and +/- 180 is backwards.
        // adjust power is how little power it uses to correct its heading.
        // millis is a time limit on the method in milliseconds.
        // buffer is the click distance from the target position at which the robot will begin slowing down.
        // negative clicks will reverse the direction. please don't use negative clicks.
        // 100 clicks is about 2 inches if the robot is going forward.
        double start = (int)System.currentTimeMillis();
        // keep heading and degrees in the correct range
        if (heading >= 180) {
            heading -= 360;
        }
        degrees -= 45;
        if (degrees >= 180) {
            degrees -= 360;
        } else if (degrees < -180) {
            degrees += 360;
        }

        // calculate difference between heading to be maintained and the direction to move towards.
        double theta = (heading-degrees)/180*Math.PI;
        if (theta >= Math.PI) {
            theta -= Math.PI*2;
        } else if (theta < -Math.PI) {
            theta += Math.PI*2;
        }
        double x_vector = Math.cos(theta);  // calculate ratio of x distance to y distance.
        double y_vector = Math.sin(theta);


        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // the mecanum wheels work in pairs, with a pair being a wheel and the wheel across the robot.
        // thus wheels 1 and 4 are one pair, the 'y' pair, and wheels 2 and 3 are the 'x' pair.
        // due to the way trigonometric functions and mecanum wheels work, it's helpful to think of the wheel pairs as x and y components.
        int xclicks = (int)Math.round(clicks*x_vector);
        int yclicks = (int)Math.round(clicks*y_vector);
        wheel1.setTargetPosition(yclicks);
        wheel2.setTargetPosition(xclicks);
        wheel3.setTargetPosition(xclicks);
        wheel4.setTargetPosition(yclicks);

        wheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheel1.setPower(speed*y_vector); // front motors are older than the back ones and require more power.
        wheel2.setPower(speed*x_vector);
        wheel3.setPower(speed*x_vector*1.05);
        wheel4.setPower(speed*y_vector*1.05);
        // loop to make robot move and continually correct heading while moving.
        double difference, sign = 1;
        while ((wheel1.isBusy() || wheel2.isBusy()) && (wheel3.isBusy() || wheel4.isBusy()) && !this.program.isStopRequested()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            // complicated logic to make the robot correct if it turns a bit off heading.
            difference = angles.firstAngle-heading;
            if (difference < 0) { sign = -1;
            } else { sign = 1; }
            if (Math.abs(difference) > 180) {
                difference = sign * (360-Math.abs(difference));
                difference = -(difference)/adjustPower;
            } else {
                difference = (difference)/adjustPower;
            }
            if (Math.abs(wheel1.getCurrentPosition()) >= Math.abs(yclicks - buffer*y_vector) ||
                    Math.abs(wheel2.getCurrentPosition()) >= Math.abs(xclicks - buffer*x_vector) ||
                    Math.abs(wheel3.getCurrentPosition()) >= Math.abs(xclicks - buffer*x_vector) ||
                    Math.abs(wheel4.getCurrentPosition()) >= Math.abs(yclicks - buffer*y_vector) ) {
                speed = 0.3;
            }

            // stop the robot from the wiggle of death
            double speed1 = difference, speed2 = difference; // speed1 is for the front wheels 1 and 2, and speed2 is for the back wheels 3 and 4.
            if (0.12 <= Math.abs(difference) && Math.abs(difference) < 0.22) {
                // only the back wheels move, meaning that the robot can turn but at a lower speed.
                speed2 = sign * 0.22;
                speed1 = 0;
            } else if (Math.abs(difference) < 0.12) {
                // at a certain threshold you'll get no movement, but the motors will whine. thus, it's best to just stop them.
                speed1 = 0;
                speed2 = 0;
            }
            // where the powers are actually adjusted.
            wheel1.setPower((speed*y_vector)+speed1);
            wheel2.setPower((speed*x_vector)-speed1);
            wheel3.setPower((speed*x_vector)+speed2);
            wheel4.setPower((speed*y_vector)-speed2);
            if (start+millis < (int)System.currentTimeMillis()) {
                break;
            }
        }
        // stop.
        wheel1.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
        wheel4.setPower(0);
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void odStrafe(double heading, double speed, double x, double y) {
        odStrafe(heading, speed, x, y, 1, 0.02, 120000);
    }
    public void odStrafe(double heading, double speed, double x, double y, double buffer) {
        odStrafe(heading, speed, x, y, buffer, 0.02, 120000);
    }
    public void odStrafe(double heading, double speed, double x, double y, double buffer, double adjustPower) {
        odStrafe(heading, speed, x, y, buffer, adjustPower, 120000);
    }
    public void odStrafe(double heading, double speed, double x, double y, double buffer, double adjustPower, int millis) {
        // travels facing heading going at speed towards the point x, y (x and y are in inches).
        // buffer is how close (in inches) the robot has to get to its target position before it can move on.
        // adjust power is how much power it uses to correct its heading. Values should be be between 0.2-0.0001
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean offTarget = true;
        double start = (int)System.currentTimeMillis();
        while (offTarget && !this.program.isStopRequested()) {
            ArrayList<Double> list = odometer.getCurrentCoordinates();
            double angle = list.get(0);
            double current_x = list.get(1);
            double current_y = list.get(2);

            double xdis = x - current_x;
            double ydis = y - current_y;
            if (Math.abs(xdis) + Math.abs(ydis) <= Math.abs(buffer)) {
                offTarget = false;
                continue;
            }
            double degrees = Math.atan2(-xdis, ydis);  // ironically, degrees
            // (the direction the robot is supposed to go towards) is in radians.

            double theta = ((heading/180)*Math.PI) - degrees + Math.PI/4;
            if (theta >= Math.PI) {
                theta -= Math.PI*2;
            } else if (theta < -Math.PI) {
                theta += Math.PI*2;
            }
            double x_vector = Math.cos(theta);  // calculate ratio of x distance to y distance.
            double y_vector = Math.sin(theta);

            // complicated logic to make the robot correct if it turns a bit off heading.
            double sign;
            double difference = angle-heading;
            if (Math.abs(difference) > 180) {
                if (difference < 0) { sign = -1;
                } else { sign = 1; }
                difference = sign * (360-Math.abs(difference));
                difference = -(difference)*adjustPower;
            } else {
                difference = (difference)*adjustPower;
            }
            wheel1.setPower((speed*y_vector)+difference);
            wheel2.setPower((speed*x_vector)-difference);
            wheel3.setPower((speed*x_vector)+difference);
            wheel4.setPower((speed*y_vector)-difference);
            if (start+millis < (int)System.currentTimeMillis()) {
                break;
            }
        }
        // stop
        wheel2.setPower(0);
        wheel4.setPower(0);
        wheel1.setPower(0);
        wheel3.setPower(0);
    }
    public void odTurn(int degrees, double speed, int millis) {
        // has robot turn at speed power to heading degrees. degrees should be 180 >= degrees >= -180.
        // millis is a time limit on the method in milliseconds.
        double difference, sign, correct, final_speed, start = (int)System.currentTimeMillis();
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double angle;
        ArrayList<Double> list = odometer.getCurrentCoordinates();
        angle = list.get(0);
        while (angle != degrees && !this.program.isStopRequested()) {
            list = odometer.getCurrentCoordinates();
            angle = list.get(0);
            difference = angle-degrees;
            if (difference < 0) { sign = -1;
            } else { sign = 1; }

            // make sure that the robot turn the correct direction
            if (Math.abs(difference) > 180) {
                difference = sign * (360-Math.abs(difference));
                correct = -(difference)/120;
                if (Math.abs(correct) > 1) { correct = -sign; } // make sure that we don't correct so mauch that we give the motors greater power than speed.
            } else {
                correct = (difference)/120;
                if (Math.abs(correct) > 1) { correct = sign; } // make sure that we don't correct so mauch that we give the motors greater power than speed.
            }
            final_speed = speed*correct;
            if (Math.abs(final_speed) < 0.05) {
                if (final_speed > 0) {final_speed = 0.05;}
                if (final_speed < 0) {final_speed = -0.05;}
            }
            wheel2.setPower(-final_speed);
            wheel4.setPower(-final_speed);
            wheel1.setPower(final_speed);
            wheel3.setPower(final_speed);
            if (start+millis < (int)System.currentTimeMillis()) {
                break;
            }
        }
        wheel2.setPower(0);
        wheel4.setPower(0);
        wheel1.setPower(0);
        wheel3.setPower(0);
    }
    public void odSleep(int mls) {
        odometer.odSleep(mls);
    }
}
