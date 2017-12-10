package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

	Servo input1;
	Servo input2;
	Servo input3;
	Servo input4;

	ColorSensor revColor;
	ColorSensor MRColor;

	Telemetry telemetry;

	private static final int ticksPerInch=89;


	private ElapsedTime runtime = new ElapsedTime();

	public void initRobot(HardwareMap spareMap, Telemetry tempTelemetry) {
		Telemetry telemetry = tempTelemetry;
		frontLeft = spareMap.dcMotor.get("front left wheel");
		frontRight = spareMap.dcMotor.get("front right wheel");
		backLeft = spareMap.dcMotor.get("back left wheel");
		backRight = spareMap.dcMotor.get("back right wheel");
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		input1=spareMap.get(Servo.class, "input1");
		input2=spareMap.get(Servo.class, "input2");
		input3=spareMap.servo.get("output1");
		input4=spareMap.servo.get("output2");
		revColor=spareMap.colorSensor.get("REV Color Sensor");
		MRColor=spareMap.colorSensor.get("MR Color Sensor");
	}

	public void setToStartIntake ()
	{
		input1.setPosition(0);
		input2.setPosition(1);
		input3.setPosition(0);
		input4.setPosition(1);
	}

	public void setToReverseIntake()
	{
		input1.setPosition(1);
		input2.setPosition(0);
		input3.setPosition(1);
		input4.setPosition(0);
	}

	public void stopIntake ()
	{
		input1.setPosition(.5);
		input2.setPosition(.5);
		input3.setPosition(.5);
		input4.setPosition(.5);
	}
	
	//TODO: make one of these for every direction, or one that works for all for all direction
	//TODO: implement acceleration and deceleration into this to prevent sliding
	public void driveForwardDistance(double power, int distance)
	{
		setDriveMotorMode("RUN_TO_POSITION");
		int frontLDist, frontRDist, backLDist, backRDist;
		//89 is the ratio for ticks per inches
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
					frontLDist>300 &&
					frontRDist>300 &&
					backLDist>300 &&
					backRDist>300
				);
		setToStill();
	}

	public void setDriveMotorMode(String mode)
	{
		switch(mode)
		{
			case "RUN_USING_ENCODER":
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_USING_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);break;
			case "RUN_WITHOUT_ENCODER":
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);break;
			case "STOP_AND_RESET_ENCODER":
				if(frontLeft.getMode()==DcMotor.RunMode.STOP_AND_RESET_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);break;
			case "RUN_TO_POSITION":
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_TO_POSITION)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);break;
		}
	}

	public void waiter(int time)
	{
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
	public void setToForward(double power) {
		frontLeft.setPower(power);
		frontRight.setPower(power);
		backLeft.setPower(power);
		backRight.setPower(power);
	}
	public void setToBackward(double power) {
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
	public void setToLeft(double power) {
		frontLeft.setPower(1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(1 * power);
	}
	public void setToRight(double power) {
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