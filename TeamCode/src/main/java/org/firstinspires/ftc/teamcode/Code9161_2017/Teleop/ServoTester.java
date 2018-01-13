package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Aledo HS Robotics on 4/12/2017.
 * gamepad1-
 * 			tank drive with joysticks
 * 			strafe with dpad
 *			precise control mode while holding down left bumper
 * gamepad2-
 * 			pulley control with dpad, up for up, down for down
 * 			flicker control with triggers, launch with right
 * 			beaconPusher control with x and b, x for left, b for right
 * 			collector control with a and y, a for take in, y for spit out
 * 			cap ball container control with dpad, left for release, right for holding
 */

@TeleOp(name="ServoTester", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled     //Determines if the program shows up on Driver Station


public class ServoTester extends OpMode
{
	Servo trayServo;
	double trayPosition=.943;

	Servo jewelPusher;
	double jewelPosition=.19;

	Servo relicServo;
	double relicPosition=.69;

	//ColorSensor MRColor;

	DcMotor lift1;
	DcMotor lift2;
    int liftPosition=0;
	public void init()
	{
		trayServo=hardwareMap.servo.get("trayServo");
		jewelPusher=hardwareMap.servo.get("jewelPusher");
		relicServo=hardwareMap.servo.get("relicServo");
		lift1=hardwareMap.dcMotor.get("lift1");
		lift2=hardwareMap.dcMotor.get("lift2");
		lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift1.setPower(1);
		lift2.setPower(1);
		/*MRColor=hardwareMap.colorSensor.get("MR Color Sensor");
		MRColor.enableLed(true);*/
	}

	public void initLoop(){}

	public void start(){}

	public void loop()
	{
		trayPosition+=gamepad1.a?.01:0;
		trayPosition-=gamepad1.y?.01:0;
		trayPosition=trayPosition>1?1:trayPosition;
		trayPosition=trayPosition<0?0:trayPosition;
		trayServo.setPosition(trayPosition);
		telemetry.addData("Tray Servo Position", trayPosition);

		if(gamepad1.right_trigger>.25)
			liftPosition+=20;
		else if(gamepad1.left_trigger>.25)
			liftPosition-=20;
		lift1.setTargetPosition(liftPosition);
		lift2.setTargetPosition(liftPosition);
        telemetry.addData("lift position", liftPosition);

		jewelPosition+=gamepad1.b?.01:0;
		jewelPosition-=gamepad1.x?.01:0;
		jewelPosition=jewelPosition>1?1:jewelPosition;
		jewelPosition=jewelPosition<0?0:jewelPosition;
     	jewelPusher.setPosition(jewelPosition);
		telemetry.addData("JewelPusher Servo Position", jewelPosition);

		/*telemetry.addData("Red: ", MRColor.red());
		telemetry.addData("Blue: ", MRColor.blue());*/

		relicPosition+=gamepad1.dpad_down?.01:0;
		relicPosition-=gamepad1.dpad_up?.01:0;
		relicPosition=relicPosition>1?1:relicPosition;
		relicPosition=relicPosition<-1?-1:relicPosition;
		relicServo.setPosition(relicPosition);
		telemetry.addData("relic Servo Position", relicPosition);
	}
}
