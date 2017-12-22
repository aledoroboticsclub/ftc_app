package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Created by Aledo HS Robotics on 4/12/2017.
 */
public class Strider {

	DcMotor frontLeft;
	DcMotor frontRight;
	DcMotor backLeft;
	DcMotor backRight;
	DcMotor lift1;
	DcMotor lift2;

	DcMotor inputMR;
	DcMotor inputML;
	Servo inputSR;
	Servo inputSL;
	Servo trayServo;

	/*ColorSensor revColor;
	ColorSensor MRColor;

	GyroSensor gyro;*/

	Telemetry telemetry;

	private static final int ticksPerInch=89;//TODO: test this value, this one is from last year
	private static final double initialTrayPosition=.092;
	private static final double placementTrayPosition=.578;
	private static final int encoderSafeZone=300;/*a motor must be within this many ticks of its
	target to be considered "on target"*/
	//in motor ticks
	private static final int liftPosition0=0;//loading position
	private static final int liftPosition1=0;//places on the ground
	private static final int liftPosition2=0;//places over 1 cubes
	private static final int liftPosition3=0;//places over 2 cubes
	private static final int liftPosition4=0;//places over 3 cubes



	private ElapsedTime runtime = new ElapsedTime();

	public void initRobot(HardwareMap spareMap, Telemetry tempTelemetry) {
		Telemetry telemetry = tempTelemetry;
		frontLeft = spareMap.dcMotor.get("front left wheel");
		frontRight = spareMap.dcMotor.get("front right wheel");
		backLeft = spareMap.dcMotor.get("back left wheel");
		backRight = spareMap.dcMotor.get("back right wheel");
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

		lift1=spareMap.dcMotor.get("lift1");
		lift2=spareMap.dcMotor.get("lift2");
		lift2.setDirection(DcMotorSimple.Direction.REVERSE);
		lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		inputML=spareMap.dcMotor.get("inputML");
		inputMR=spareMap.dcMotor.get("inputMR");
		inputML.setDirection(DcMotorSimple.Direction.REVERSE);
		inputML.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		inputMR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		inputSL=spareMap.servo.get("inputSL");
		inputSR=spareMap.servo.get("inputSR");

		trayServo=spareMap.servo.get("trayServo");

		/*revColor=spareMap.colorSensor.get("REV Color Sensor");
		MRColor=spareMap.colorSensor.get("MR Color Sensor");
		gyro=spareMap.gyroSensor.get("gyro");*/

		//setTrayToIntake();
	}

	public void setToStartIntake (double power) {
		inputML.setPower(power);
		inputMR.setPower(power);
		inputSL.setPosition(1);
		inputSR.setPosition(0);
	}
	public void setToReverseIntake() {
		inputML.setPower(-1);
		inputMR.setPower(-1);
		inputSL.setPosition(0);
		inputSR.setPosition(1);
	}
	public void stopIntake () {
		inputML.setPower(0);
		inputMR.setPower(0);
		inputSL.setPosition(.5);
		inputSR.setPosition(.5);
	}

	public void setTrayToIntake() {
		trayServo.setPosition(initialTrayPosition);
	}
	public void setTrayToPlace() {
		trayServo.setPosition(placementTrayPosition);
	}

	public void setLiftToUp(double power) {
		lift1.setPower(power);
		lift2.setPower(power);
	}
	public void setLiftToDown(double power) {
		lift1.setPower(power*-1);
		lift2.setPower(power*-1);
	}
	public void setLiftToStill() {
		lift1.setPower(0);
		lift2.setPower(0);
	}
	
	public void setLiftToPosition0() {
		lift1.setTargetPosition(liftPosition0);
		lift2.setTargetPosition(liftPosition0);
		telemetry.addData("lift position: ", "0");
		telemetry.addData("ready to load", "");
	}
	public void setLiftToPosition1() {
		lift1.setTargetPosition(liftPosition1);
		lift2.setTargetPosition(liftPosition1);
		telemetry.addData("lift position: ", "1");
		telemetry.addData("ready to place on ground", "");
	}
	public void setLiftToPosition2() {
		lift1.setTargetPosition(liftPosition2);
		lift2.setTargetPosition(liftPosition2);
		telemetry.addData("lift position: ", "2");
		telemetry.addData("ready to place over 1 cube", "");
	}
	public void setLiftToPosition3() {
		lift1.setTargetPosition(liftPosition3);
		lift2.setTargetPosition(liftPosition3);
		telemetry.addData("lift position: ", "3");
		telemetry.addData("ready to place over 2 cubes", "");
	}
	public void setLiftToPosition4() {
		lift1.setTargetPosition(liftPosition4);
		lift2.setTargetPosition(liftPosition4);
		telemetry.addData("lift position: ", "4");
		telemetry.addData("ready to place over 2 cubes", "");
	}
	
	//TODO: make one of these for every direction, or one that works for all for all direction
	//TODO: implement acceleration and deceleration into this to prevent sliding
	public void driveForwardDistance(double power, int distance)
	{
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		frontLeft.setTargetPosition(Math.round(distance*ticksPerInch)+frontLeft.getCurrentPosition());
		frontRight.setTargetPosition(Math.round(distance*ticksPerInch)+frontRight.getCurrentPosition());
		backLeft.setTargetPosition(Math.round(distance*ticksPerInch)+backLeft.getCurrentPosition());
		backRight.setTargetPosition(Math.round(distance*ticksPerInch)+backRight.getCurrentPosition());

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

	public void waiter(int time) {
		runtime.reset();
		while(runtime.milliseconds()<time){}
	}

	//driveTime methods	
	//TODO: consider implementing acceleration and deceleration into the driveTime methods, even though it may be more work than it is worth
	public void driveForwardTime(double power, int time) {
		setToForward(power);
		waiter(time);
		setToStill();
	}
	public void driveBackwatdTime(double power, int time) {
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
	//TODO: there may be a more eficient way to write these methods 
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
}