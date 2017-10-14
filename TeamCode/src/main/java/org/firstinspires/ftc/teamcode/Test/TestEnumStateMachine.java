package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Data Force Team 6929 on 10/12/2017.
 * modified by Karter on 10/13/2017
 */

@TeleOp(name="Test Enum", group="Testing")
//@Disabled
public class TestEnumStateMachine extends LinearOpMode{

    private boolean goToDefaultStage = false;

    private enum Stage{
        stage1,
        stage2,
        stage3;
    }
    private Stage mStage;

    String output;

    @Override
    public void runOpMode() {

        mStage = Stage.stage1;
        waitForStart();

        boolean DONE = false;
        while (!DONE) {
            switch (mStage){
                case stage1:
                    output+="Stage 1";
                    mStage = Stage.stage2;
                    break;
                case stage2:
                    output+=", Stage 2";
                    if (goToDefaultStage){
                        mStage = Stage.stage3;
                    } else {
                        mStage = Stage.stage1;
                        goToDefaultStage = true;
                    }
                    break;
                case stage3:
                    output+=", Stage 3";
                    telemetry.update();
                    DONE = true;
                    // Done
                    break;
            }

        }

        output+="Done";
        while (opModeIsActive()) {
            telemetry.addData(output,"");
            telemetry.update();
        }

    }

}

/*switch (mStage) {
            case 1:   //drive forward here
                    output+=" FWD";
                    break;
                    case 2:  //turn here
                    output+=" TURN";
                    break;
                    case 3: //sense here
                    output+=" SENSE";
                    break;
                    case 4:
                    output+=" BWD";
                    break;
default://stop or something here
        output+=" DEFAULT";
        break;

        }

        */
