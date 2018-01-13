<<<<<<< HEAD
package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by Aledo HS Robotics on 4/12/2017.
 */
public class Scorpion {

	DcMotor frontLeft;
	DcMotor frontRight;
	DcMotor backLeft;
	DcMotor backRight;
	DcMotor lift1;
	DcMotor lift2;
	DcMotor inputMR;
	DcMotor inputML;

	Servo trayServo;
	Servo jewelPusher;
	Servo relicGrabber;
	Servo extender;

	ColorSensor MRColor;
	GyroSensor gyroSensor;
	ModernRoboticsI2cGyro gyro;
	int zAccumulated;

	Telemetry telemetry;
	HardwareMap hardwareMap;

	public static final String TAG = "Vuforia Navigation";
	OpenGLMatrix lastLocation = null;
	VuforiaLocalizer vuforia;

	ElapsedTime timer = new ElapsedTime();

	private static final double initialTrayPosition=.943;
	private static final double parallelTrayPosistion=.673;
	private static final double placementTrayPosition=0.06;

	private static final double jewelPusherDownPosition=1;
	private static final double jewelPusherUpPosition=.32;

	private static final double relicGrabberGrabbedPosition=.6;
	private static final double relicGrabberReleasePosition=.45;

	private static final int ticksPerInch=89;//TODO: test this value, this one is from last year
	private static final int encoderSafeZone=300;/*a motor must be within this many ticks of its
	target to be considered "on target"*/

	//in motor ticks
	private static final int liftPosition0=0;//loading position, and places on the ground
	private static final int liftPosition1=440;//places over 1 cubes
	private static final int liftPosition2=2360;//places over 2 cubes
	private static final int liftPosition3=3000;//places over 3 cubes
	private static final double liftPower=1;//max speed lift travels at, currently unused
	private static final double liftSpeedMultiplier=80;//modifies the speed of manual control of lift


	public void initRobot(HardwareMap spareMap, Telemetry tempTelemetry) {
		telemetry = tempTelemetry;
		hardwareMap=spareMap;

		frontLeft = hardwareMap.dcMotor.get("front left wheel");
		frontRight = hardwareMap.dcMotor.get("front right wheel");
		backLeft = hardwareMap.dcMotor.get("back left wheel");
		backRight = hardwareMap.dcMotor.get("back right wheel");
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

		lift1=hardwareMap.dcMotor.get("lift1");
		lift2=hardwareMap.dcMotor.get("lift2");
		lift2.setDirection(DcMotorSimple.Direction.REVERSE);
		lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift1.setPower(liftPower);
		lift2.setPower(liftPower);

		inputML=hardwareMap.dcMotor.get("inputML");
		inputMR=hardwareMap.dcMotor.get("inputMR");
		inputML.setDirection(DcMotorSimple.Direction.REVERSE);
		inputML.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		inputMR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		trayServo=hardwareMap.servo.get("trayServo");
		relicGrabber=hardwareMap.servo.get("relicServo");
		jewelPusher=hardwareMap.servo.get("jewelPusher");
		extender=hardwareMap.servo.get("extender");

		setTrayToParallel();
		jewelPusher.setPosition(jewelPusherUpPosition);
		setGrabberToGrabbed();

		MRColor=hardwareMap.colorSensor.get("MR Color Sensor");
		MRColor.enableLed(true);
	}

	public void rightIntake (double power) {
		inputMR.setPower(power*.4);
	}
	public void leftIntake (double power) {
		inputML.setPower(power*-.7);
	}

	public void setTrayToIntake() {
		trayServo.setPosition(initialTrayPosition);
	}
	public void setTrayToPlace() {
		trayServo.setPosition(placementTrayPosition);
	}
	public void setTrayToParallel() {trayServo.setPosition(parallelTrayPosistion);}

	public void extenderOut(){
		extender.setPosition(1);
	}
	public void extenderIn() {
		extender.setPosition(0);
	}
	public void extenderStill(){
		extender.setPosition(.5);
	}

	public void setGrabberToGrabbed(){relicGrabber.setPosition(relicGrabberGrabbedPosition);}
	public void setGrabberToRelease(){relicGrabber.setPosition(relicGrabberReleasePosition);}

	public void moveLiftDown(double power) {//power is a value 0 to 1
		lift1.setTargetPosition(lift1.getCurrentPosition()+((int) Math.round(power * liftSpeedMultiplier)));
		lift2.setTargetPosition(lift2.getCurrentPosition()+((int) Math.round(power * liftSpeedMultiplier)));
	}
	public void moveLiftUp(double power) {
		lift1.setTargetPosition(lift1.getCurrentPosition()-((int) Math.round(power * liftSpeedMultiplier)));
		lift2.setTargetPosition(lift2.getCurrentPosition()-((int) Math.round(power * liftSpeedMultiplier)));
	}

	public void setLiftToPosition0() {
		lift1.setTargetPosition(liftPosition0);
		lift2.setTargetPosition(liftPosition0);
		telemetry.addData("lift position: ", "0");
		telemetry.addData("ready to load\nready to place on ground", "");
	}
	public void setLiftToPosition1() {
		lift1.setTargetPosition(liftPosition1);
		lift2.setTargetPosition(liftPosition1);
		telemetry.addData("lift position: ", "1");
		telemetry.addData("ready to place over 1 cube", "");
	}
	public void setLiftToPosition2() {
		lift1.setTargetPosition(liftPosition2);
		lift2.setTargetPosition(liftPosition2);
		telemetry.addData("lift position: ", "2");
		telemetry.addData("ready to place over 2 cubes", "");
	}
	public void setLiftToPosition3() {
		lift1.setTargetPosition(liftPosition3);
		lift2.setTargetPosition(liftPosition3);
		telemetry.addData("lift position: ", "3");
		telemetry.addData("ready to place over 3 cubes", "");
	}

	//TODO: implement acceleration and deceleration into this to prevent sliding
	public void driveForwardEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(-distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(-distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
					frontLDist>encoderSafeZone &&
					frontRDist>encoderSafeZone &&
					backLDist>encoderSafeZone &&
					backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void driveBackwardEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(-distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(-distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void driveLeftEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(-distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(-distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(-distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(-distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void driveRightEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void turnClockwiseEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(-distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(-distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void turnCounterwiseEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(-distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(-distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}

	public void setDriveMotorMode(DcMotor.RunMode mode) {
		switch(mode)
		{
			case RUN_USING_ENCODER:
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_USING_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);break;
			case RUN_WITHOUT_ENCODER:
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);break;
			case STOP_AND_RESET_ENCODER:
				if(frontLeft.getMode()==DcMotor.RunMode.STOP_AND_RESET_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);break;
			case RUN_TO_POSITION:
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_TO_POSITION)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);break;
		}
	}
	public void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
		switch(behavior) {
			case BRAKE:
				if(frontLeft.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.BRAKE)
					break;
				frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);break;
			case FLOAT:
				if(frontLeft.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.FLOAT)
					break;
				frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);break;
		}
	}
	public void setDriveMotorPower(double power){
		frontLeft.setPower(power);
		frontRight.setPower(power);
		backLeft.setPower(power);
		backRight.setPower(power);
	}//for use with driveEncoder methods
	public void waiter(int time) {
		timer.reset();
		while(timer.milliseconds()<time){}
	}

	//jewel pusher method
	public void pushJewel(String teamColor){
		jewelPusher.setPosition(jewelPusherDownPosition);
		MRColor.enableLed(false);
		MRColor.enableLed(true);
		waiter(1000);
		if(teamColor.equals("Blue")){
			if(MRColor.blue()>MRColor.red()){
				telemetry.addData("Blue","");
				turnClockwiseEncoder(1,3);
				jewelPusher.setPosition(jewelPusherUpPosition);
				turnCounterwiseEncoder(1,3);
			}
			else{
				telemetry.addData("Red","");
				turnCounterwiseEncoder(1,3);
				jewelPusher.setPosition(jewelPusherUpPosition);
				turnClockwiseEncoder(1,3);
			}
		}
		if(teamColor == "Red"){
			if(MRColor.red()>MRColor.blue()){
				telemetry.addData("Red","");
				turnClockwiseEncoder(1,3);
				jewelPusher.setPosition(jewelPusherUpPosition);
				turnCounterwiseEncoder(1,3);
			}
			else{
				telemetry.addData("Blue","");
				turnCounterwiseEncoder(1,3);
				jewelPusher.setPosition(jewelPusherUpPosition);
				turnClockwiseEncoder(1,3);
			}
		}
		waiter(500);
	}

	//driveTime methods
	//TODO: consider implementing acceleration and deceleration into the driveTime methods, even though it may be more work than it is worth
	public void driveForwardTime(double power, int time) {
		setToForward(power);
		waiter(time);
		setToStill();
	}
	public void driveBackwardTime(double power, int time) {
		setToBackward(power);
		waiter(time);
		setToStill();
	}
	public void driveClockwiseTime(double power, int time) {
		setToClockwise(power);
		waiter(time);
		setToStill();
	}
	public void driveCounterwiseTime(double power, int time) {
		setToCounterwise(power);
		waiter(time);
		setToStill();
	}
	public void driveLeftTime(double power, int time) {
		setToLeft(power);
		waiter(time);
		setToStill();
	}
	public void driveRightTime(double power, int time) {
		setToRight(power);
		waiter(time);
		setToStill();
	}

	//setTo methods
	//TODO: there may be a more efficient way to write these methods
	public void setToRight(double power) {
		frontLeft.setPower(power);
		frontRight.setPower(power);
		backLeft.setPower(power);
		backRight.setPower(power);
	}
	public void setToLeft(double power) {
		frontLeft.setPower(-1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(-1 * power);
	}
	public void setToClockwise(double power) {
		frontLeft.setPower(-1 * power);
		frontRight.setPower(1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(1 * power);
	}
	public void setToCounterwise(double power) {
		frontLeft.setPower(1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(1 * power);
		backRight.setPower(-1 * power);
	}
	public void setToForward(double power) {
		frontLeft.setPower(1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(1 * power);
	}
	public void setToBackward(double power) {
		frontLeft.setPower(-1 * power);
		frontRight.setPower(1 * power);
		backLeft.setPower(1 * power);
		backRight.setPower(-1 * power);
	}
	public void setToStill() {
		frontLeft.setPower(0);
		frontRight.setPower(0);
		backLeft.setPower(0);
		backRight.setPower(0);
	}

	//TODO: must change so that it can work in SET_TO_POSTITION mode, maybe one for both modes
	public void turnAbsolute(int target, double power) {
		setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
		zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings

		while (Math.abs(zAccumulated - target)  >0 )//Continue while the robot direction is further than three degrees from the target
		{
			telemetry.addData("heading: ",gyro.getHeading());
			telemetry.addData("integratedZValue: ",gyro.getIntegratedZValue());
			telemetry.update();
			if (zAccumulated > target)//if gyro is positive, turn counterwise
			{
				frontLeft.setPower(-power);
				frontRight.setPower(power);
				backLeft.setPower(-power);
				backRight.setPower(power);
			}

			if (zAccumulated < target)//if gyro is negative, turn clockwise
			{
				frontLeft.setPower(power);
				frontRight.setPower(-power);
				backLeft.setPower(power);
				backRight.setPower(-power);
			}
			zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings
			telemetry.addData("accu", String.format("%03d", zAccumulated));
			telemetry.update();
		}
		setToStill();
	}
	public int decodePictograph(){
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		parameters.vuforiaLicenseKey = "Ac8Q0nb/////AAAAGZRBQQCid0gNt3ydtz8W8gB8MrlkYn+Gu+jvldH+Igx9SypXvRwUWJw/71iF8xhpjKXBDv1UDD+EsjkvvC1+Zko/hF+lZG/TglT50MCsw6/q2MuSc+AUFDqT9lhEJcyroMMp20VPNwj/fUoUAxr5DV4+VUdwwYW/sCML6iL/x0rWEzUGxJf8qvKSrZcI/4X2fWsryCaprTXecsZCTudHQiElph2GCtMva4843D9sx+a6NB9zhPiyn6aaydEs5T4Ygc5o2nK1p6o8G82++XtlDYPkBuVBajLsO6z0Zvk980xIWmgyKjMNZlLofM7lLJdjt5Sh4a1imlIlsAWbQqPvs35MxJLmmrugrO7WXXveK4TY";

		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

		VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
		VuforiaTrackable relicTemplate = relicTrackables.get(0);
		relicTemplate.setName("relicVuMarkTemplate");
		RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
		if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
			telemetry.addData("VuMark", "%s visible", vuMark);
			if(vuMark == RelicRecoveryVuMark.LEFT)
				return 0;
			if(vuMark == RelicRecoveryVuMark.CENTER)
				return 6;
			if(vuMark == RelicRecoveryVuMark.RIGHT)
				return 12;
		}
		else {
			telemetry.addData("VuMark", "not visible");
		}
		telemetry.update();
		return 0;
	}
}
=======
package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by Aledo HS Robotics on 4/12/2017.
 */
public class Scorpion {

	DcMotor frontLeft;
	DcMotor frontRight;
	DcMotor backLeft;
	DcMotor backRight;
	DcMotor lift1;
	DcMotor lift2;
	DcMotor inputMR;
	DcMotor inputML;

	Servo trayServo;
	Servo jewelPusher;
	Servo relicGrabber;
	Servo extender;

	ColorSensor MRColor;



	Telemetry telemetry;
	HardwareMap hardwareMap;

	public static final String TAG = "Vuforia VuMark";
	OpenGLMatrix lastLocation = null;
	VuforiaLocalizer vuforia;

	ElapsedTime timer = new ElapsedTime();

	private static final double initialTrayPosition=.943;
	private static final double parallelTrayPosistion=.673;
	private static final double placementTrayPosition=0.06;

	private static final double jewelPusherDownPosition=.81;
	private static final double jewelPusherUpPosition=.19;

	private static final double relicGrabberGrabbedPosition=.6;
	private static final double relicGrabberReleasePosition=.45;

	private static final int ticksPerInch=89;//TODO: test this value, this one is from last year
	private static final int encoderSafeZone=300;/*a motor must be within this many ticks of its
	target to be considered "on target"*/

	//in motor ticks
	private static final int liftPosition0=0;//loading position, and places on the ground
	private static final int liftPosition1=440;//places over 1 cubes
	private static final int liftPosition2=2360;//places over 2 cubes
	private static final int liftPosition3=3000;//places over 3 cubes
	private static final double liftPower=1;//max speed lift travels at, currently unused
	private static final double liftSpeedMultiplier=80;//modifies the speed of manual control of lift


	public void initRobot(HardwareMap spareMap, Telemetry tempTelemetry) {
		telemetry = tempTelemetry;
		hardwareMap=spareMap;

		frontLeft = hardwareMap.dcMotor.get("front left wheel");
		frontRight = hardwareMap.dcMotor.get("front right wheel");
		backLeft = hardwareMap.dcMotor.get("back left wheel");
		backRight = hardwareMap.dcMotor.get("back right wheel");
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

		lift1=hardwareMap.dcMotor.get("lift1");
		lift2=hardwareMap.dcMotor.get("lift2");
		lift2.setDirection(DcMotorSimple.Direction.REVERSE);
		lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift1.setPower(liftPower);
		lift2.setPower(liftPower);

		inputML=hardwareMap.dcMotor.get("inputML");
		inputMR=hardwareMap.dcMotor.get("inputMR");
		inputML.setDirection(DcMotorSimple.Direction.REVERSE);
		inputML.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		inputMR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		trayServo=hardwareMap.servo.get("trayServo");
		relicGrabber=hardwareMap.servo.get("relicServo");
		jewelPusher=hardwareMap.servo.get("jewelPusher");
		extender=hardwareMap.servo.get("extender");

		setTrayToIntake();
		jewelPusher.setPosition(jewelPusherUpPosition);
		setGrabberToGrabbed();

		/*MRColor=hardwareMap.colorSensor.get("MR Color Sensor");
		MRColor.enableLed(true);*/
	}

	public void rightIntake (double power) {
		inputMR.setPower(power*.4);
	}
	public void leftIntake (double power) {
		inputML.setPower(power*-.7);
	}

	public void setTrayToIntake() {
		trayServo.setPosition(initialTrayPosition);
	}
	public void setTrayToPlace() {
		trayServo.setPosition(placementTrayPosition);
	}
	public void setTrayToParallel() {trayServo.setPosition(parallelTrayPosistion);}

	public void extenderOut(){
		extender.setPosition(1);
	}
	public void extenderIn() {
		extender.setPosition(0);
	}
	public void extenderStill(){
		extender.setPosition(.5);
	}

	public void setGrabberToGrabbed(){relicGrabber.setPosition(relicGrabberGrabbedPosition);}
	public void setGrabberToRelease(){relicGrabber.setPosition(relicGrabberReleasePosition);}

	public void moveLiftDown(double power) {//power is a value 0 to 1
		lift1.setTargetPosition(lift1.getCurrentPosition()+((int) Math.round(power * liftSpeedMultiplier)));
		lift2.setTargetPosition(lift2.getCurrentPosition()+((int) Math.round(power * liftSpeedMultiplier)));
	}
	public void moveLiftUp(double power) {
		lift1.setTargetPosition(lift1.getCurrentPosition()-((int) Math.round(power * liftSpeedMultiplier)));
		lift2.setTargetPosition(lift2.getCurrentPosition()-((int) Math.round(power * liftSpeedMultiplier)));
	}

	public void setLiftToPosition0() {
		lift1.setTargetPosition(liftPosition0);
		lift2.setTargetPosition(liftPosition0);
		telemetry.addData("lift position: ", "0");
		telemetry.addData("ready to load\nready to place on ground", "");
	}
	public void setLiftToPosition1() {
		lift1.setTargetPosition(liftPosition1);
		lift2.setTargetPosition(liftPosition1);
		telemetry.addData("lift position: ", "1");
		telemetry.addData("ready to place over 1 cube", "");
	}
	public void setLiftToPosition2() {
		lift1.setTargetPosition(liftPosition2);
		lift2.setTargetPosition(liftPosition2);
		telemetry.addData("lift position: ", "2");
		telemetry.addData("ready to place over 2 cubes", "");
	}
	public void setLiftToPosition3() {
		lift1.setTargetPosition(liftPosition3);
		lift2.setTargetPosition(liftPosition3);
		telemetry.addData("lift position: ", "3");
		telemetry.addData("ready to place over 3 cubes", "");
	}

	//TODO: implement acceleration and deceleration into this to prevent sliding
	public void driveForwardEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(-distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(-distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
					frontLDist>encoderSafeZone &&
					frontRDist>encoderSafeZone &&
					backLDist>encoderSafeZone &&
					backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void driveBackwardEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(-distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(-distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void driveLeftEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(-distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(-distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(-distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(-distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void driveRightEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void turnClockwiseEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(-distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(-distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void turnCounterwiseEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(distance*ticksPerInch+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(-distance*ticksPerInch+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(distance*ticksPerInch+backLeft.getCurrentPosition());
		backRight.setTargetPosition(-distance*ticksPerInch+backRight.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
				frontLDist>encoderSafeZone &&
						frontRDist>encoderSafeZone &&
						backLDist>encoderSafeZone &&
						backRDist>encoderSafeZone
				);
		setToStill();
	}

	public void setDriveMotorMode(DcMotor.RunMode mode) {
		switch(mode)
		{
			case RUN_USING_ENCODER:
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_USING_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);break;
			case RUN_WITHOUT_ENCODER:
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);break;
			case STOP_AND_RESET_ENCODER:
				if(frontLeft.getMode()==DcMotor.RunMode.STOP_AND_RESET_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);break;
			case RUN_TO_POSITION:
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_TO_POSITION)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);break;
		}
	}
	public void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
		switch(behavior) {
			case BRAKE:
				if(frontLeft.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.BRAKE)
					break;
				frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);break;
			case FLOAT:
				if(frontLeft.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.FLOAT)
					break;
				frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);break;
		}
	}
	public void setDriveMotorPower(double power){
		frontLeft.setPower(power);
		frontRight.setPower(power);
		backLeft.setPower(power);
		backRight.setPower(power);
	}//for use with driveEncoder methods
	public void waiter(int time) {
		timer.reset();
		while(timer.milliseconds()<time){}
	}

	//jewel pusher method
	public void pushJewel(String teamColor){
		jewelPusher.setPosition(jewelPusherDownPosition);
		MRColor.enableLed(false);
		MRColor.enableLed(true);
		waiter(1000);
		if(teamColor.equals("Blue")){
			if(MRColor.blue()>MRColor.red()){
				telemetry.addData("Blue","");
				turnClockwiseEncoder(1,3);
				jewelPusher.setPosition(jewelPusherUpPosition);
				turnCounterwiseEncoder(1,3);
			}
			else{
				telemetry.addData("Red","");
				turnCounterwiseEncoder(1,3);
				jewelPusher.setPosition(jewelPusherUpPosition);
				turnClockwiseEncoder(1,3);
			}
		}
		if(teamColor == "Red"){
			if(MRColor.red()>MRColor.blue()){
				telemetry.addData("Red","");
				turnClockwiseEncoder(1,3);
				jewelPusher.setPosition(jewelPusherUpPosition);
				turnCounterwiseEncoder(1,3);
			}
			else{
				telemetry.addData("Blue","");
				turnCounterwiseEncoder(1,3);
				jewelPusher.setPosition(jewelPusherUpPosition);
				turnClockwiseEncoder(1,3);
			}
		}
	}

	//driveTime methods
	//TODO: consider implementing acceleration and deceleration into the driveTime methods, even though it may be more work than it is worth
	public void driveForwardTime(double power, int time) {
		setToForward(power);
		waiter(time);
		setToStill();
	}
	public void driveBackwardTime(double power, int time) {
		setToBackward(power);
		waiter(time);
		setToStill();
	}
	public void driveClockwiseTime(double power, int time) {
		setToClockwise(power);
		waiter(time);
		setToStill();
	}
	public void driveCounterwiseTime(double power, int time) {
		setToCounterwise(power);
		waiter(time);
		setToStill();
	}
	public void driveLeftTime(double power, int time) {
		setToLeft(power);
		waiter(time);
		setToStill();
	}
	public void driveRightTime(double power, int time) {
		setToRight(power);
		waiter(time);
		setToStill();
	}

	//setTo methods
	//TODO: there may be a more efficient way to write these methods
	public void setToRight(double power) {
		frontLeft.setPower(power);
		frontRight.setPower(power);
		backLeft.setPower(power);
		backRight.setPower(power);
	}
	public void setToLeft(double power) {
		frontLeft.setPower(-1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(-1 * power);
	}
	public void setToClockwise(double power) {
		frontLeft.setPower(-1 * power);
		frontRight.setPower(1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(1 * power);
	}
	public void setToCounterwise(double power) {
		frontLeft.setPower(1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(1 * power);
		backRight.setPower(-1 * power);
	}
	public void setToForward(double power) {
		frontLeft.setPower(1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(1 * power);
	}
	public void setToBackward(double power) {
		frontLeft.setPower(-1 * power);
		frontRight.setPower(1 * power);
		backLeft.setPower(1 * power);
		backRight.setPower(-1 * power);
	}
	public void setToStill() {
		frontLeft.setPower(0);
		frontRight.setPower(0);
		backLeft.setPower(0);
		backRight.setPower(0);
	}

	//TODO: must change so that it can work in SET_TO_POSTITION mode, maybe one for both modes
//	public void turnAbsolute(int target, double power) {
//		setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings
//
//		while (Math.abs(zAccumulated - target)  >0 )//Continue while the robot direction is further than three degrees from the target
//		{
//			telemetry.addData("heading: ",gyro.getHeading());
//			telemetry.addData("integratedZValue: ",gyro.getIntegratedZValue());
//			telemetry.update();
//			if (zAccumulated > target)//if gyro is positive, turn counterwise
//			{
//				frontLeft.setPower(-power);
//				frontRight.setPower(power);
//				backLeft.setPower(-power);
//				backRight.setPower(power);
//			}
//
//			if (zAccumulated < target)//if gyro is negative, turn clockwise
//			{
//				frontLeft.setPower(power);
//				frontRight.setPower(-power);
//				backLeft.setPower(power);
//				backRight.setPower(-power);
//			}
//			zAccumulated = gyro.getIntegratedZValue();  //Set variables to gyro readings
//			telemetry.addData("accu", String.format("%03d", zAccumulated));
//			telemetry.update();
//		}
//		setToStill();
//	}
	public int decodePictograph(){ //returns a distance value for the robot to travel
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		parameters.vuforiaLicenseKey = "Ac8Q0nb/////AAAAGZRBQQCid0gNt3ydtz8W8gB8MrlkYn+Gu+jvldH+Igx9SypXvRwUWJw/71iF8xhpjKXBDv1UDD+EsjkvvC1+Zko/hF+lZG/TglT50MCsw6/q2MuSc+AUFDqT9lhEJcyroMMp20VPNwj/fUoUAxr5DV4+VUdwwYW/sCML6iL/x0rWEzUGxJf8qvKSrZcI/4X2fWsryCaprTXecsZCTudHQiElph2GCtMva4843D9sx+a6NB9zhPiyn6aaydEs5T4Ygc5o2nK1p6o8G82++XtlDYPkBuVBajLsO6z0Zvk980xIWmgyKjMNZlLofM7lLJdjt5Sh4a1imlIlsAWbQqPvs35MxJLmmrugrO7WXXveK4TY";

		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

		VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
		VuforiaTrackable relicTemplate = relicTrackables.get(0);
		relicTemplate.setName("relicVuMarkTemplate");
		RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
		if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
			telemetry.addData("VuMark", "%s visible", vuMark);
			if(vuMark == RelicRecoveryVuMark.LEFT)
				return 0;
			if(vuMark == RelicRecoveryVuMark.CENTER)
				return 6;
			if(vuMark == RelicRecoveryVuMark.RIGHT)
				return 12;
		}
		else {
			telemetry.addData("VuMark", "not visible");
		}
		telemetry.update();
		return 0;
	}
}
>>>>>>> 268fe100df8689f28beb5ef9df1c2dc30592deb1
