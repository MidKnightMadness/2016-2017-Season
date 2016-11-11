package org.tka.robotics.opmode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeMeta;
import com.qualcomm.robotcore.util.ClassFilter;
import com.qualcomm.robotcore.util.ClassManager;

import java.lang.reflect.Field;
import java.util.*;

public class RedBlueOpModeRegistrar implements ClassFilter {
    private static final String TAG = "FTC_OP_MODE_REGISTER::";


    private static List<Class<RedBlueOpMode>> toRegister = new ArrayList<>();

    private static List<String> toReg = new ArrayList<>();

    private final OpModeManager register;

    private RedBlueOpModeRegistrar() {
        register = null;
    }

    private RedBlueOpModeRegistrar(OpModeManager register) {
        this.register = register;
    }

    /**
     * This gets the correct name of the OpMode via reflection. This returns the value of the name
     * present in the Annotation or Simple Name of the class.
     *
     * @param opMode the OpMode to check
     * @return the name to use of defined by the user
     */
    private static String getOpModeName(Class opMode) {
        String name;
        if (opMode.isAnnotationPresent(RedBlueAutonomous.class)) {
            name = ((RedBlueAutonomous) opMode.getAnnotation(RedBlueAutonomous.class)).name();
        } else {
            name = opMode.getSimpleName();
        }

        if (name.equals("")) {
            name = opMode.getSimpleName();
        }
        return name;
    }

    /**
     * This statically loads OpModes via reflection. This is the main method for this procedure
     *
     * @param register the Ftc OpMode Register
     */
    public static void register(OpModeManager register) {
        try {
            ClassManager m = new ClassManager();
            m.registerFilter(new RedBlueOpModeRegistrar());
            m.processAllClasses();
            RedBlueOpModeRegistrar r = new RedBlueOpModeRegistrar(register);
            r.process();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void process() {
        if(register == null)
            return;
        try {
            for (Class<RedBlueOpMode> clazz : toRegister) {
                String opModeName = getOpModeName(clazz);
                OpMode redOpMode = clazz.newInstance();
                OpMode blueOpMode = clazz.newInstance();
                Field teamColorField = clazz.getField("teamColor");
                teamColorField.setAccessible(true);
                teamColorField.set(redOpMode, TeamColor.RED);
                teamColorField.set(blueOpMode, TeamColor.BLUE);
                this.register.register(new OpModeMeta(String.format("[R] %s", opModeName), OpModeMeta.Flavor.AUTONOMOUS, "RedOpModes"), redOpMode);
                this.register.register(new OpModeMeta(String.format("[B] %s", opModeName), OpModeMeta.Flavor.AUTONOMOUS, "BlueOpModes"), blueOpMode);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void filter(Class aClass) {
        if (aClass.isAnnotationPresent(RedBlueAutonomous.class)) {
            if (aClass.isAnnotationPresent(Disabled.class)) {
                return;
            }
            if (RedBlueOpMode.class.isAssignableFrom(aClass)) {
                if (!toReg.contains(aClass.getCanonicalName())) {
                    Log.i(TAG, String.format("The class {%s} has met the requirements, registering", aClass.getCanonicalName()));
                    toReg.add(aClass.getCanonicalName());
                    toRegister.add((Class<RedBlueOpMode>) aClass);
                }
            } else
                Log.w(TAG, String.format("Class {%s} is not a RedBlueOpMode", aClass.getCanonicalName()));
        }
    }
}