// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;

import java.util.List;

/** Add your docs here. */
public class Interpolate {
    List<Double> xA;
    List<Double> yA;
    int cnt = 0;
    public Interpolate(List<Double> _x, List<Double> _y){
        xA = _x;
        yA = _y;
    }
    public double calculate(double x){
        double rtn = 0;
        if(xA.size() != yA.size()) {
            return 0;
        }
        if(x < xA.get(0)){
            return yA.get(0);
        }else if(x > xA.get(yA.size()-1)){
            return yA.get(yA.size()-1);
        }
        
        for (int i = 0; i < xA.size()-1; i++) {
            if(x > xA.get(i) && x < xA.get(i+1)){
                Double x1 = xA.get(i);
                Double x2 = xA.get(i + 1);

                Double y1 = yA.get(i);
                Double y2 = yA.get(i + 1);

                Double xDis = x2 - x1;
                Double yDis = y2 - y1;

                Double xPer = (x - xA.get(i)) /xDis;
                return rtn = yDis * xPer + y1;

            }
        }

        return rtn;
    }
    

}
