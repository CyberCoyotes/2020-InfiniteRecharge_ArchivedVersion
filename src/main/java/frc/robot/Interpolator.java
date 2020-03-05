/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Interpolator {
    static double angles[] = {13.06595, 10.573, 8.05872, 6.2501, 4.949104, 2.73799, 1.858061, 1.481606};
    static double speeds[] = {13000, 13000, 13000, 12700, 12700, 12800, 14000, 16000, 17200};

    static double getInterpolation(double angle) {
        int i = 0;
        while(angles[i] > angle) {
            i++;
            if(i >= angles.length) {
                return 0;
            }
        }
        if(i == 0) {
            return speeds[i];
        }

        int i1 = i-1;
        int i2 = i;
        double m = (speeds[i2]-speeds[i1])/(angles[i2]-angles[i1]);
        double b = speeds[i1] - m*angles[i1];
        double interpolation = m*angle+b;
        return interpolation;
    }
}
