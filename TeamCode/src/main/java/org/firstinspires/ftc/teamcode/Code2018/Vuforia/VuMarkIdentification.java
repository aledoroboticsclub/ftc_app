package org.firstinspires.ftc.teamcode.Code2018.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Code2018.Scorpion;
/**
 * Created by jreynoldsstudent on 11/25/2017.
 */
@Autonomous(name="VuMarkIdentification", group ="Concept")
//@Disabled
public class VuMarkIdentification extends LinearOpMode
{
    Scorpion r=new Scorpion();
    @Override public void runOpMode() {

        r.getOpmodeVariables(hardwareMap, telemetry);
        r.initVuforia();

        waitForStart();

        while (opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(r.relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                telemetry.addData("VuMark", "%s visible", vuMark);
            else
                telemetry.addData("VuMark", "not visible");

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}


