package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Aledo HS Robotics on 1/31/2017.
 */

public class Methods9161
{
	/* Declare OpMode members. */
	private ElapsedTime runtime=new ElapsedTime();
	//Driving Variables
	DcMotor frontLeftDrive;
	DcMotor frontRightDrive;
	DcMotor backLeftDrive;
	DcMotor backRightDrive;
	double rPower=0;
	double lPower=0;

	int aTimes=0;
	//Ball Pincher Variables
	DcMotor pinch;
	double pinchPow;
	boolean pinchLock=false;
	int pinchPWMcount=0;
	//Lift Variable
	DcMotor rightLift;
	DcMotor leftLift;
	double xTimes=0;
	double rLiftPow=0;
	double lLiftPow=0;
	//flicker variable
	DcMotor flicker;
	double flickPow;
	boolean flickerPressed=false;
	//beacon servos
	Servo rightPuncher;
	Servo leftPuncher;
	Servo ballRelease;
	double rightPuncherSpeed=0.5;
	double leftPuncherSpeed=0.5;
	double BRdelta=.01;
	double BRPos=.5;

	Servo liftHolder;
	Servo topGrabber;
	byte[] color3cache;//bottom
	byte[] color4cache;//side
	I2cDevice c3;//bottom
	I2cDevice c4;//side
	I2cDeviceSynch color3reader;//bottom
	I2cDeviceSynch color4reader;//side
	int color3Number=0;//bottom
	int color4Number=0;//side
	boolean LEDState=true;
	OpticalDistanceSensor dSense1;//leftPuncher sensor
	OpticalDistanceSensor dSense2;//rightPuncher sensor
	Telemetry telemetry;
	int zAccumulated;  //Total rotation left/right
	int target = 0;  //Desired angle to turn to
	// General Gyro Sensor allows us to point to the sensor in the configuration file.
	GyroSensor gyroSensor;
	ModernRoboticsI2cGyro mrGyro;
	int frontLeftDriveTargetPosition;
	int frontRightDriveTargetPosition;
	int backLeftDriveTargetPosition;
	int backRightDriveTargetPosition;
	LinearOpMode ourOpMode;
	I2cDevice rangeSensor;
	I2cDeviceSynch rangeReader;
	byte[] rangeCache;

	public void outputGyroData()
	{
		telemetry.addData("heading: ",mrGyro.getHeading());
		telemetry.addData("integratedZ: ",mrGyro.getIntegratedZValue());
		telemetry.update();
	}

	public void outputColorData()
	{
		telemetry.update();

	}

	public void outputRangeData()
	{
		rangeCache=rangeReader.read(0x04,2);
		telemetry.addData("rangeCache: \n",rangeCache.toString());
	}

	public void outputOpticalDistanceData()
	{
		telemetry.update();
	}

	public void driveForwardDistance(double power, int distance)
	{
		setDriveMotorMode("RUN_TO_POSITION");
		int frontLeftDriveDistanceFrom;
		int frontRightDriveDistanceFrom;
		int backLeftDriveDistanceFrom;
		int backRightDriveDistanceFrom;
		frontLeftDriveTargetPosition+=Math.round(distance*89);//89 is the ratio for ticks per inches
		frontRightDriveTargetPosition+=Math.round(distance*89);
		backLeftDriveTargetPosition+=Math.round(distance*89);
		backRightDriveTargetPosition+=Math.round(distance*89);
		frontLeftDrive.setTargetPosition(frontLeftDriveTargetPosition);
		frontRightDrive.setTargetPosition(frontRightDriveTargetPosition);
		backLeftDrive.setTargetPosition(backLeftDriveTargetPosition);
		backRightDrive.setTargetPosition(backRightDriveTargetPosition);

		frontLeftDrive.setPower(power);
		frontRightDrive.setPower(power);
		backLeftDrive.setPower(power);
		backRightDrive.setPower(power);
		do{
			frontLeftDriveDistanceFrom=Math.abs(frontLeftDrive.getTargetPosition()-frontLeftDrive.getCurrentPosition());
			frontRightDriveDistanceFrom=Math.abs(frontRightDrive.getTargetPosition()-frontRightDrive.getCurrentPosition());
			backLeftDriveDistanceFrom=Math.abs(backLeftDrive.getTargetPosition()-backLeftDrive.getCurrentPosition());
			backRightDriveDistanceFrom=Math.abs(backRightDrive.getTargetPosition()-backRightDrive.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLeftDriveDistanceFrom);
			telemetry.addData("frontRight distanceFrom: ",frontRightDriveDistanceFrom);
			telemetry.addData("backLeft distanceFrom: ",backLeftDriveDistanceFrom);
			telemetry.addData("backRight distanceFrom: ",backRightDriveDistanceFrom);
			telemetry.update();
		}while(
				(frontLeftDriveDistanceFrom>300 &&
				frontRightDriveDistanceFrom>300 &&
				backLeftDriveDistanceFrom>300 &&
				backRightDriveDistanceFrom>300)
				);
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
		frontLeftDriveTargetPosition=frontLeftDrive.getCurrentPosition();
		frontRightDriveTargetPosition=frontRightDrive.getCurrentPosition();
		backLeftDriveTargetPosition=backLeftDrive.getCurrentPosition();
		backRightDriveTargetPosition=backRightDrive.getCurrentPosition();
	}

	public void driveBackwardDistance(double power, int distance)
	{
		setDriveMotorMode("RUN_TO_POSITION");
		int frontLeftDriveDistanceFrom;
		int frontRightDriveDistanceFrom;
		int backLeftDriveDistanceFrom;
		int backRightDriveDistanceFrom;
		frontLeftDriveTargetPosition-=Math.round(distance*89);//89 is the ratio for ticks per inches
		frontRightDriveTargetPosition-=Math.round(distance*89);
		backLeftDriveTargetPosition-=Math.round(distance*89);
		backRightDriveTargetPosition-=Math.round(distance*89);
		frontLeftDrive.setTargetPosition(frontLeftDriveTargetPosition);
		frontRightDrive.setTargetPosition(frontRightDriveTargetPosition);
		backLeftDrive.setTargetPosition(backLeftDriveTargetPosition);
		backRightDrive.setTargetPosition(backRightDriveTargetPosition);

		frontLeftDrive.setPower(power);
		frontRightDrive.setPower(power);
		backLeftDrive.setPower(power);
		backRightDrive.setPower(power);
		do{
			frontLeftDriveDistanceFrom=Math.abs(frontLeftDrive.getTargetPosition()-frontLeftDrive.getCurrentPosition());
			frontRightDriveDistanceFrom=Math.abs(frontRightDrive.getTargetPosition()-frontRightDrive.getCurrentPosition());
			backLeftDriveDistanceFrom=Math.abs(backLeftDrive.getTargetPosition()-backLeftDrive.getCurrentPosition());
			backRightDriveDistanceFrom=Math.abs(backRightDrive.getTargetPosition()-backRightDrive.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLeftDriveDistanceFrom);
			telemetry.addData("frontRight distanceFrom: ",frontRightDriveDistanceFrom);
			telemetry.addData("backLeft distanceFrom: ",backLeftDriveDistanceFrom);
			telemetry.addData("backRight distanceFrom: ",backRightDriveDistanceFrom);
			telemetry.update();
		}while(
						frontLeftDriveDistanceFrom>300 &&
						frontRightDriveDistanceFrom>300 &&
						backLeftDriveDistanceFrom>300 &&
						backRightDriveDistanceFrom>300
				);
		frontLeftDriveTargetPosition=frontLeftDrive.getCurrentPosition();
		frontRightDriveTargetPosition=frontRightDrive.getCurrentPosition();
		backLeftDriveTargetPosition=backLeftDrive.getCurrentPosition();
		backRightDriveTargetPosition=backRightDrive.getCurrentPosition();
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
	}

	public void turnCounterwiseDistance(double power, int distance)
	{
		setDriveMotorMode("RUN_TO_POSITION");
		int frontLeftDriveDistanceFrom;
		int frontRightDriveDistanceFrom;
		int backLeftDriveDistanceFrom;
		int backRightDriveDistanceFrom;
		frontLeftDriveTargetPosition-=Math.round(distance*12.5);//12.5 is the ratio for ticks per degrees
		frontRightDriveTargetPosition+=Math.round(distance*12.5);
		backLeftDriveTargetPosition-=Math.round(distance*12.5);
		backRightDriveTargetPosition+=Math.round(distance*12.5);
		frontLeftDrive.setTargetPosition(frontLeftDriveTargetPosition);
		frontRightDrive.setTargetPosition(frontRightDriveTargetPosition);
		backLeftDrive.setTargetPosition(backLeftDriveTargetPosition);
		backRightDrive.setTargetPosition(backRightDriveTargetPosition);

		frontLeftDrive.setPower(power);
		frontRightDrive.setPower(power);
		backLeftDrive.setPower(power);
		backRightDrive.setPower(power);
		do{
			frontLeftDriveDistanceFrom=Math.abs(frontLeftDrive.getTargetPosition()-frontLeftDrive.getCurrentPosition());
			frontRightDriveDistanceFrom=Math.abs(frontRightDrive.getTargetPosition()-frontRightDrive.getCurrentPosition());
			backLeftDriveDistanceFrom=Math.abs(backLeftDrive.getTargetPosition()-backLeftDrive.getCurrentPosition());
			backRightDriveDistanceFrom=Math.abs(backRightDrive.getTargetPosition()-backRightDrive.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLeftDriveDistanceFrom);
			telemetry.addData("frontRight distanceFrom: ",frontRightDriveDistanceFrom);
			telemetry.addData("backLeft distanceFrom: ",backLeftDriveDistanceFrom);
			telemetry.addData("backRight distanceFrom: ",backRightDriveDistanceFrom);
			telemetry.update();
		}while(
				frontLeftDriveDistanceFrom>0 &&
						frontRightDriveDistanceFrom>0 &&
						backLeftDriveDistanceFrom>0 &&
						backRightDriveDistanceFrom>0
				);
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
	}

	public void turnClockwiseDistance(double power, int distance)
	{
		setDriveMotorMode("RUN_TO_POSITION");
		int frontLeftDriveDistanceFrom;
		int frontRightDriveDistanceFrom;
		int backLeftDriveDistanceFrom;
		int backRightDriveDistanceFrom;
		frontLeftDriveTargetPosition+=(distance*12);//12.5 is the ratio for ticks per degrees
		frontRightDriveTargetPosition-=(distance*12);
		backLeftDriveTargetPosition+=(distance*12);
		backRightDriveTargetPosition-=(distance*12);
		frontLeftDrive.setTargetPosition(frontLeftDriveTargetPosition);
		frontRightDrive.setTargetPosition(frontRightDriveTargetPosition);
		backLeftDrive.setTargetPosition(backLeftDriveTargetPosition);
		backRightDrive.setTargetPosition(backRightDriveTargetPosition);

		frontLeftDrive.setPower(power);
		frontRightDrive.setPower(power);
		backLeftDrive.setPower(power);
		backRightDrive.setPower(power);
		do{
			frontLeftDriveDistanceFrom=Math.abs(frontLeftDrive.getTargetPosition()-frontLeftDrive.getCurrentPosition());
			frontRightDriveDistanceFrom=Math.abs(frontRightDrive.getTargetPosition()-frontRightDrive.getCurrentPosition());
			backLeftDriveDistanceFrom=Math.abs(backLeftDrive.getTargetPosition()-backLeftDrive.getCurrentPosition());
			backRightDriveDistanceFrom=Math.abs(backRightDrive.getTargetPosition()-backRightDrive.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLeftDriveDistanceFrom);
			telemetry.addData("frontRight distanceFrom: ",frontRightDriveDistanceFrom);
			telemetry.addData("backLeft distanceFrom: ",backLeftDriveDistanceFrom);
			telemetry.addData("backRight distanceFrom: ",backRightDriveDistanceFrom);
			telemetry.update();
		}while(
				frontLeftDriveDistanceFrom>0 &&
						frontRightDriveDistanceFrom>0 &&
						backLeftDriveDistanceFrom>0 &&
						backRightDriveDistanceFrom>0
				);
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
	}

	public void turnRelative(int turnDegree, double power)
	{
		setDriveMotorMode("RUN_USING_ENCODER");
		int startingDegree=mrGyro.getIntegratedZValue();

		if(turnDegree>0)//if positive go counterwise
		{
			while(startingDegree+turnDegree>mrGyro.getIntegratedZValue()+3)
			{
				frontLeftDrive.setPower(-power);
				frontRightDrive.setPower(power);
				backLeftDrive.setPower(-power);
				backRightDrive.setPower(power);
			}
		}
		else{//otherwise go clockwise
			while(startingDegree-turnDegree<mrGyro.getIntegratedZValue()-3 )
			{
				frontLeftDrive.setPower(power);
				frontRightDrive.setPower(-power);
				backLeftDrive.setPower(power);
				backRightDrive.setPower(-power);
			}
		}
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
	}

	public void turnAbsolute(int target, double power)
	{
		setDriveMotorMode("RUN_USING_ENCODER");
		zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings

		while (Math.abs(zAccumulated - target)  >0 )//Continue while the robot direction is further than three degrees from the target
		{
			telemetry.addData("heading: ",mrGyro.getHeading());
			telemetry.addData("integratedZValue: ",mrGyro.getIntegratedZValue());
			telemetry.update();
			if (zAccumulated > target)//if gyro is positive, turn counterwise
			{
				frontLeftDrive.setPower(-power);
				frontRightDrive.setPower(power);
				backLeftDrive.setPower(-power);
				backRightDrive.setPower(power);
			}

			if (zAccumulated < target)//if gyro is negative, turn clockwise
			{
				frontLeftDrive.setPower(power);
				frontRightDrive.setPower(-power);
				backLeftDrive.setPower(power);
				backRightDrive.setPower(-power);
			}
			zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
			telemetry.addData("accu", String.format("%03d", zAccumulated));
			telemetry.update();
		}
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
	}

	public void turnAbsolute(int target, double power, String direction)//yet to be tested
	{
		setDriveMotorMode("RUN_USING_ENCODER");
		zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings

		while (Math.abs(zAccumulated - target )  >0 )//Continue while the robot direction is further than three degrees from the target
		{
			telemetry.addData("heading: ",mrGyro.getHeading());
			telemetry.addData("integratedZValue: ",mrGyro.getIntegratedZValue());
			telemetry.update();
			if (direction.equals("counterwise"))//turn counterwise
			{
				frontLeftDrive.setPower(-power);
				frontRightDrive.setPower(power);
				backLeftDrive.setPower(-power);
				backRightDrive.setPower(power);
			}

			else//turn clockwise
			{
				frontLeftDrive.setPower(power);
				frontRightDrive.setPower(-power);
				backLeftDrive.setPower(power);
				backRightDrive.setPower(-power);
			}
			zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
			telemetry.addData("accu", String.format("%03d", zAccumulated));
			telemetry.update();
		}
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
	}

	public void driveForwardTillWhite(double power)
	{
		int timeTilOveride=3000;
		setDriveMotorMode("RUN_USING_ENCODER");
		frontLeftDrive.setPower(power);
		frontRightDrive.setPower(power);
		backLeftDrive.setPower(power);
		backRightDrive.setPower(power);
		runtime.reset();
		do
		{
			color3cache=color3reader.read(0x04,1);
			color3Number=color3cache[0]&0xFF;
		}while(color3Number==0 && runtime.milliseconds()<timeTilOveride);
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
	}

	public void driveBackwardTillWhite(double power)
	{
		int timeTilOveride=3000;
		setDriveMotorMode("RUN_USING_ENCODER");
		frontLeftDrive.setPower(-power);
		frontRightDrive.setPower(-power);
		backLeftDrive.setPower(-power);
		backRightDrive.setPower(-power);

		do
		{
			color3cache=color3reader.read(0x04,1);
			color3Number=color3cache[0]&0xFF;
		}while(color3Number==0&& runtime.milliseconds()<timeTilOveride);
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
	}

	public void setDriveMotorMode(String mode)//not finished the modes must still be explained
	{
		switch(mode)
		{
			case "RUN_USING_ENCODER":
				if(frontLeftDrive.getMode()==DcMotor.RunMode.RUN_USING_ENCODER)
					break;
				frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);break;
			case "RUN_WITHOUT_ENCODER":
				if(frontLeftDrive.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER)
					break;
				frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);break;
			case "STOP_AND_RESET_ENCODER":
				if(frontLeftDrive.getMode()==DcMotor.RunMode.STOP_AND_RESET_ENCODER)
					break;
				frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);break;
			case "RUN_TO_POSITION":
				if(frontLeftDrive.getMode()==DcMotor.RunMode.RUN_TO_POSITION)
					break;
				frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);break;
		}
	}

	public void initRobot(HardwareMap spareMap, Telemetry tempTelemetry)
	{
		frontLeftDrive=spareMap.dcMotor.get("front left wheel");
		frontRightDrive=spareMap.dcMotor.get("front right wheel");
		backLeftDrive=spareMap.dcMotor.get("back left wheel");
		backRightDrive=spareMap.dcMotor.get("back right wheel");
		pinch=spareMap.dcMotor.get("pincher");
		rightLift=spareMap.dcMotor.get("right lift");
		leftLift=spareMap.dcMotor.get("left lift");
		flicker=spareMap.dcMotor.get("ball flicker");
		rightPuncher=spareMap.servo.get("left puncher");
		leftPuncher=spareMap.servo.get("right puncher");
		ballRelease=spareMap.servo.get("ball release");
		liftHolder=spareMap.servo.get("lift holder");
		topGrabber=spareMap.servo.get("scorpion");
		frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		setDriveMotorMode("STOP_AND_RESET_ENCODER");
		setDriveMotorMode("RUN_TO_POSITION");
		c3=spareMap.i2cDevice.get("c3");//bottom
		c4=spareMap.i2cDevice.get("c4");//side
		color3reader=new I2cDeviceSynchImpl(c3,I2cAddr.create8bit(0x3c),false);//bottom
		color4reader=new I2cDeviceSynchImpl(c4,I2cAddr.create8bit(0x4c),false);//side
		color3reader.engage();//bottom
		color4reader.engage();//side
		color3reader.write8(3,0);//bottom
		color4reader.write8(3,1);//side
		boolean isOnWhite=false;
		liftHolder.setPosition(1);
		dSense1 = spareMap.opticalDistanceSensor.get("dSense1");//leftPuncher sensor
		dSense2 = spareMap.opticalDistanceSensor.get("dSense2");//rightPuncher sensor
		telemetry=tempTelemetry;
		gyroSensor = spareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
		mrGyro=(ModernRoboticsI2cGyro)gyroSensor;//ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
		gyroSensor.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID
		while (gyroSensor.isCalibrating()) {} //Ensure calibration is complete (usually 2 seconds)
		rangeSensor=spareMap.i2cDevice.get("range");
		rangeReader=new I2cDeviceSynchImpl(rangeSensor,I2cAddr.create8bit(0x28),false);
		rangeReader.engage();
	}

	public void theFlickThing()
	{
		int launchTime=500;
		flicker.setPower(1);
		waiter(launchTime);
		flicker.setPower(0);
		ballRelease.setPosition(.8);
		waiter(450);
		ballRelease.setPosition(.5);
		flicker.setPower(1);
		waiter(launchTime);
		flicker.setPower(0);
	}

	public void leftPunch()
	{
		int distanceFromBeacon;
		int additionalExtensionTime=700;
		int overExtensionTime=2800;

		leftPuncher.setPosition(1);
		runtime.reset();

		do
		{
			if(runtime.milliseconds()>overExtensionTime)//servo has extended to far, abort
				break;
			distanceFromBeacon=(int)Math.round(dSense1.getRawLightDetected()*100);
		}while(distanceFromBeacon<35);//until we are close to the beacon
		waiter(additionalExtensionTime);

		leftPuncher.setPosition(0);

		/*double waitTemp=runtime.milliseconds();
		runtime.reset();
		while(runtime.milliseconds()<waitTemp){}//should bring servo in for the same time it extended*/
		waiter(2500);
		leftPuncher.setPosition(.5);
	}

	public void rightPunch()
	{
		int distanceFromBeacon;
		int additionalExtensionTime=700;
		int overExtensionTime=2800;

		rightPuncher.setPosition(0);
		runtime.reset();

		do
		{
			if(runtime.milliseconds()>overExtensionTime)//servo has extended to far, abort
				break;
			distanceFromBeacon=(int)Math.round(dSense2.getRawLightDetected()*100);
		}while(distanceFromBeacon<35);//until we are close to the beacon
		waiter(additionalExtensionTime);

		rightPuncher.setPosition(1);

		/*double waitTemp=runtime.milliseconds();
		runtime.reset();
		while(runtime.milliseconds()<waitTemp){}//should bring servo in for the same time it extended*/
		waiter(2500);
		rightPuncher.setPosition(.5);
	}

	public void beaconTIMEBlue()
	{
		color4cache=color4reader.read(0x04,1);
		color4Number=color4cache[0]&0xFF;
		telemetry.addData("color4Number",color4Number);
		telemetry.update();
		waiter(500);
		telemetry.addData("color4Number",color4Number);
		telemetry.update();
		if(color4Number>=5)//if beacon is red
		{
			telemetry.addData("color4Number",color4Number);
			telemetry.addData("ColorSensed","red");
			telemetry.update();
			rightPunch();
		}else{
			telemetry.addData("color4Number",color4Number);
			telemetry.addData("ColorSensed","blue");
			telemetry.update();
			leftPunch();
		}
	}

	public void beaconTIMERed()
	{
		color4cache=color4reader.read(0x04,1);
		color4Number=color4cache[0]&0xFF;
		telemetry.addData("color4Number",color4Number);
		telemetry.update();
		waiter(500);
		telemetry.addData("color4Number",color4Number);
		telemetry.update();
		if(color4Number>=5)//if beacon is red
		{
			telemetry.addData("color4Number",color4Number);
			telemetry.addData("ColorSensed","red");
			telemetry.update();
			leftPunch();
		}else{
			telemetry.addData("color4Number",color4Number);
			telemetry.addData("ColorSensed","blue");
			telemetry.update();
			rightPunch();
		}
	}

	public void waiter(int time)
	{
		runtime.reset();
		while(runtime.milliseconds()<time){}
	}

	public void driveForwardTime(double power,int time)//does not use encoders, time in milliseconds
	{
		if(true)
		{
			setDriveMotorMode("RUN_USING_ENCODER");
			backLeftDrive.setPower(.01);
			frontLeftDrive.setPower(.01);
			frontRightDrive.setPower(.01);
			backRightDrive.setPower(.01);
			waiter(1);

			backLeftDrive.setPower(power);
			frontLeftDrive.setPower(power);
			frontRightDrive.setPower(power);
			backRightDrive.setPower(power);

			waiter(time);
			frontLeftDrive.setPower(0);
			frontRightDrive.setPower(0);
			backLeftDrive.setPower(0);
			backRightDrive.setPower(0);
		}
	}

	public void driveBackwardTime(double power,int time)//does not use encoders, time in milliseconds
	{
		if(true)
		{
			setDriveMotorMode("RUN_USING_ENCODER");
			backLeftDrive.setPower(.01);
			frontLeftDrive.setPower(.01);
			frontRightDrive.setPower(.01);
			backRightDrive.setPower(.01);
			waiter(1);
			frontLeftDrive.setPower(-power);
			frontRightDrive.setPower(-power);
			backLeftDrive.setPower(-power);
			backRightDrive.setPower(-power);
			waiter(time);
			frontLeftDrive.setPower(0);
			frontRightDrive.setPower(0);
			backLeftDrive.setPower(0);
			backRightDrive.setPower(0);
		}
	}

	public void turnClockwiseTime(double power,int time)//does not use encoders, time in milliseconds
	{
		if(true)
		{
			setDriveMotorMode("RUN_USING_ENCODER");
			backLeftDrive.setPower(.01);
			frontLeftDrive.setPower(.01);
			frontRightDrive.setPower(.01);
			backRightDrive.setPower(.01);
			waiter(1);
			frontLeftDrive.setPower(-power);
			frontRightDrive.setPower(power);
			backLeftDrive.setPower(-power);
			backRightDrive.setPower(power);
			waiter(time);
			frontLeftDrive.setPower(0);
			frontRightDrive.setPower(0);
			backLeftDrive.setPower(0);
			backRightDrive.setPower(0);
		}
	}

	public void turnCounterwiseTime(double power,int time)//does not use encoders, time in milliseconds
	{
		if(true)
		{
			setDriveMotorMode("RUN_USING_ENCODER");
			backLeftDrive.setPower(.01);
			frontLeftDrive.setPower(.01);
			frontRightDrive.setPower(.01);
			backRightDrive.setPower(.01);
			waiter(1);
			frontLeftDrive.setPower(power);
			frontRightDrive.setPower(-power);
			backLeftDrive.setPower(power);
			backRightDrive.setPower(-power);
			waiter(time);
			frontLeftDrive.setPower(0);
			frontRightDrive.setPower(0);
			backLeftDrive.setPower(0);
			backRightDrive.setPower(0);
		}
	}




}

