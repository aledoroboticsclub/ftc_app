package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name="Basic: Linear OpMode", group="Linear Opmode")
public class AutonomousBlue1 extends LinearOpMode {
    Scorpion r = new Scorpion();
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            r.pushJewel("Blue");
            r.driveForwardEncoder(1,12);
            r.vuforia();
            r.driveRightEncoder(1,12);

        }
    }
}
