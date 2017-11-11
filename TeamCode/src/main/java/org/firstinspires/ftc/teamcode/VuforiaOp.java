package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Alex on 2016-12-4.
 */
@Autonomous(name="SynergyVuforiaOp", group="Autonomous")
@Disabled
public class VuforiaOp extends LinearOpMode {

    static final double CENTER_OF_TURNING = 150;
    static final double     WHEEL_CIRCUMFRENCE   = 6280000 ;
    static final double     COUNTS_PER_MOTOR_REV    = 1120;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor leftMotor = hardwareMap.dcMotor.get("left_drive");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AT2rAxn/////AAAAGbnMcmWtp0xwjzdpH6wFzU1/Gi1HXOZBybiwikL7KgRiKgga6O0Ky0yu7e7HrhxM2qYNDKTi60nOV4WeI3mmfZD4PhISqYHUF3hVWfNrUnk8d+hf0ysxRu9wGaKIWzg6wvqoVBI1m3eHoZoBPFKD0olJulpU9oyRx2DBcpHvCMOGj2Ru0QY8lJs989+XgVESGVW+b/XmUpuf728eij+ohdQdFL8ZOSmDz40Fluq7dQQW3wMgqUlV3TpVoc0sp02Vl7BxZonLOKgR3/RJBuLFFM1Eq9YxjqifGLHm/DS9Sdmudqv5J48X0km4Estf/bvRojrx/GyeI+/SvUj86XYTrwbyz3hfI+9gUMENBh3go4Mq";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        waitForStart();

        beacons.activate();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setPower(0.2);
        leftMotor.setPower(0.2);

        while (opModeIsActive() && wheels.getRawPose() == null) {
            idle();
        }

        rightMotor.setPower(0.2);
        leftMotor.setPower(0.2);

        VectorF angles = anglesFromTarget(wheels);

        VectorF trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees((angles.get(0))) - 90, new VectorF(500, 0, 0));

        if (trans.get(0) > 0) {
            leftMotor.setPower(0.02);
            rightMotor.setPower(-0.02);

        } else {
            leftMotor.setPower(-0.02);
            rightMotor.setPower(0.02);

        }

        do {
            if (wheels.getPose() != null) {
                trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees((angles.get(0))) - 90, new VectorF(500, 0, 0));
            }
            idle();
        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30);

        leftMotor.setPower(0);
        rightMotor.setPower(0);


        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightMotor.setTargetPosition((int) (leftMotor.getCurrentPosition() * ((Math.hypot((trans.get(0)), trans.get(2)) + CENTER_OF_TURNING / WHEEL_CIRCUMFRENCE * COUNTS_PER_MOTOR_REV))));
        leftMotor.setTargetPosition((int) (leftMotor.getCurrentPosition() * ((Math.hypot((trans.get(0)), trans.get(2)) + CENTER_OF_TURNING / WHEEL_CIRCUMFRENCE * COUNTS_PER_MOTOR_REV))));

        leftMotor.setPower(0.3);
        rightMotor.setPower(0.3);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && (wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10)) {
            if (wheels != null) {
                if (wheels.getPose().getTranslation().get(0) > 0) {
                    leftMotor.setPower(-0.3);
                    rightMotor.setPower(0.3);
                } else {
                    leftMotor.setPower(0.3);
                    rightMotor.setPower(-0.3);
                }
            } else {
                leftMotor.setPower(-0.3);
                rightMotor.setPower(0.3);
            }


        /*        while (opModeIsActive()){
            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                }
            }
            telemetry.update();
        */
        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);



    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall) {
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));

    }
    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){ float [] data = image.getRawPose().getData(); float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}}; double thetaX = Math.atan2(rotation[2][1], rotation[2][2]); double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2])); double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]); return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }}