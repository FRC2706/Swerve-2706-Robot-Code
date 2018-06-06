package org.usfirst.frc.team2706.robot;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.lang.reflect.Array;
import java.util.Scanner;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name.
 * This provides flexibility changing wiring, makes checking the wiring easier and significantly
 * reduces the number of magic numbers floating around.
 */
@SuppressWarnings("unused")
public class RobotMap {

    private static final int ROBOT_ID = getRobotID();

    // CONSTANT_VALS [ Swerve ]

    public static final boolean INVERT_JOYSTICK_X = false;
    public static final boolean INVERT_JOYSTICK_Y = false;

    /**
     * Prints which RobotMap is being used
     */
    public static void log() {
        Log.i("RobotMap", "RobotMap ID is " + ROBOT_ID);
    }

    private static final String ROBOT_ID_LOC = "/home/lvuser/robot-type.conf";

    private static int getRobotID() {
        int temp = 0;

        try (Scanner sc = new Scanner(new BufferedReader(new FileReader(ROBOT_ID_LOC)))) {
            temp = sc.nextInt();
        } catch (FileNotFoundException e) {
            temp = 0;
        }

        return temp;
    }

    @SuppressWarnings("unchecked")
    private static <T> T getConstant(String constant) {
        try {
            return RobotConfig.get("RobotMap." + constant, (T) getArray(RobotMap.class
                            .getDeclaredField(constant + "_VALS").get(null))[ROBOT_ID]);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return null;
    }

    float boat; // must be a float or else it sinks

    private static Object[] getArray(Object val) {
        int arrlength = Array.getLength(val);
        Object[] outputArray = new Object[arrlength];

        for (int i = 0; i < arrlength; ++i) {
            outputArray[i] = Array.get(val, i);
        }

        return outputArray;
    }
}
