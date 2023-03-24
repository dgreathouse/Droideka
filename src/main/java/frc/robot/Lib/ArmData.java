// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;

import java.util.HashMap;

/** Add your docs here. */
public class ArmData {




    HashMap<String, ArmInfo> data = new HashMap<String, ArmInfo>();
    public ArmData(){          // ARM INFO( Bicep/Shoulder   Forearm/Elbow    Wrist/Intake)

        
        data.put(ArmPosEnum.HOME.toString(), new ArmInfo(-126.5,4.5));

        /******************  CONE DATA **************************/
        data.put(ArmPosEnum.WALL_CONE.toString(), new ArmInfo(2.2,  300));
        data.put(ArmPosEnum.MID_CONE.toString(), new ArmInfo(57,  158));
        data.put(ArmPosEnum.LOW_CONE.toString(), new ArmInfo(57,  158));

        /******************  CUBE DATA **************************/
        data.put(ArmPosEnum.FAR_CUBE.toString(), new ArmInfo(45,  60));
        data.put(ArmPosEnum.MID_CUBE.toString(), new ArmInfo(65, 50));
        data.put(ArmPosEnum.LOW_CUBE.toString(), new ArmInfo(-126.5,  4.5));

        data.put(ArmPosEnum.FAR_CUBE_REVERSE.toString(), new ArmInfo(-22,  91));
        data.put(ArmPosEnum.MID_CUBE_REVERSE.toString(), new ArmInfo(-52, 57));

        data.put(ArmPosEnum.FLOOR_FRONT_CUBE.toString(), new ArmInfo(-126.5, 76));

        /******************  AUTO ARM DATA **************************/
        data.put(ArmPosEnum.AUTO_FRONT_CUBE.toString(), new ArmInfo(-126.5, 76));
        data.put(ArmPosEnum.AUTO_HOME.toString(),new ArmInfo(0, 0));

    }
    public double getBicepAngle(ArmPosEnum _posString){
        return data.get(_posString.toString()).bAngle;
    }

    public double getHandAngle(ArmPosEnum _posString){
        return data.get(_posString.toString()).hAngle;
    }

public class ArmInfo{

   public double bAngle = 0;

   public double hAngle = 0;

    public ArmInfo(double shoulderAngle, double wristAngle){
        bAngle = shoulderAngle;

        hAngle = wristAngle;
    }
}

}
