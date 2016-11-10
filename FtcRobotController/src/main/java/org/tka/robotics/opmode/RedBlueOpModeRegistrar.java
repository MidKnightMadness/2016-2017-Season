/*
 * Copyright © 2016 David Sargent
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation  the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM,OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package org.tka.robotics.opmode;

import android.app.Application;
import android.content.Context;
import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeMeta;
import com.qualcomm.robotcore.util.RobotLog;
import dalvik.system.DexFile;

import java.io.IOException;
import java.lang.annotation.Annotation;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.*;

public class RedBlueOpModeRegistrar {
    private static final String TAG = "FTC_OP_MODE_REGISTER::";

    private final LinkedList<String> noCheckList;

    private RedBlueOpModeRegistrar() {
        noCheckList = new LinkedList<>();
    }

    /**
     * This gets the correct name of the OpMode via reflection. This returns the value of the name
     * present in the Annotation or Simple Name of the class.
     *
     * @param opMode the OpMode to check
     * @return the name to use of defined by the user
     */
    private static String getOpModeName(Class<OpMode> opMode) {
        String name;
        if (opMode.isAnnotationPresent(RedBlueAutonomous.class)) {
            name = opMode.getAnnotation(RedBlueAutonomous.class).name();
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
    public static void loadOpModes(OpModeManager register) {
        RedBlueOpModeRegistrar me = new RedBlueOpModeRegistrar();
        me.buildNoCheckList();

        LinkedList<Class> classes = me.buildClassList();
        HashMap<String, LinkedList<Class<OpMode>>> opModes = me.findOpModes(classes);
        me.sortOpModeMap(opModes);

        LinkedList<Class<OpMode>> opModesToRegister = new LinkedList<>();
        for (LinkedList<Class<OpMode>> opModeList : me.treeMapify(opModes).values()) {
            for (Class<OpMode> opMode : opModeList) {

                // todo workaround an the issue of reflectively loading an non-public class
                // fails silenty
                if (!Modifier.isPublic(opMode.getModifiers())) {
                    RobotLog.setGlobalErrorMsg(getOpModeName(opMode) + " is marked as an OpMode" +
                            ", however it is not public. Please add the keyword 'public' to it");
                }
                // register.register(getOpModeName(opMode), opMode);
                opModesToRegister.add(opMode);
            }
        }

        for (Class<OpMode> opMode : opModesToRegister) {
            RedBlueOpMode redOpMode, blueOpMode;
            try {
                if(!(opMode.newInstance() instanceof RedBlueOpMode)){
                    Log.w(TAG, String.format("The class {%s} is not a RedBlueOpMode, skipping", opMode.getName()));
                    continue;
                }
                redOpMode = (RedBlueOpMode) opMode.newInstance();
                redOpMode.teamColor = TeamColor.RED;
                blueOpMode = (RedBlueOpMode) opMode.newInstance();
                redOpMode.teamColor = TeamColor.BLUE;
            } catch (Exception e) {
                Log.e(TAG, String.format("Exception thrown when attempting to instance %s", opMode.getCanonicalName()), e);
                continue;
            }
            Log.i(TAG, String.format("Registering Red & Blue variations of %s", opMode.getName()));
            String opModeName;
            if (opMode.isAnnotationPresent(RedBlueAutonomous.class))
                opModeName = opMode.getAnnotation(RedBlueAutonomous.class).name();
            else
                opModeName = opMode.getSimpleName();
            register.register(new OpModeMeta("[R] " + opModeName, OpModeMeta.Flavor.AUTONOMOUS, "RedAutoModes"), redOpMode);
            register.register(new OpModeMeta("[B] " + opModeName, OpModeMeta.Flavor.AUTONOMOUS, "BlueAutoModes"), blueOpMode);
        }

    }

    /**
     * Uses a context generated by {@link #getApplicationContext()} to get a list of the class
     * packages within in this application
     *
     * @return a list of classes present and that can be used
     */
    private LinkedList<Class> buildClassList() {
        DexFile df;
        Context applicationContext = getApplicationContext();

        // Try to load the Dex-File
        try {
            df = new DexFile(applicationContext.getPackageCodePath());
        } catch (IOException e) {
            Log.wtf(TAG, e);
            throw new NullPointerException();
        }


        // Migrate the enum to OpMode LinkedList
        LinkedList<String> classes = new LinkedList<>(Collections.list(df.entries()));

        // Create the container for the objects to add
        LinkedList<Class> classesToProcess = new LinkedList<>();
        for (String klazz : classes) {
            if (shouldAdd(klazz)) {
                try {
                    classesToProcess.add(Class.forName(klazz, false, applicationContext.getClassLoader()));
                } catch (ClassNotFoundException | NoClassDefFoundError e) {
                    Log.e(TAG, "Cannot add class to process list: " + klazz + "Reason: " + e.toString());

                    // Add code to prevent loading the next class
                    if (klazz.contains("$")) {
                        klazz = klazz.substring(0, klazz.indexOf("$") - 1);
                    }
                    noCheckList.add(klazz);
                }
            }
        }

        return classesToProcess;
    }

    /**
     * Creates a list of packages not to check to improve errors and reduce logging
     */
    private void buildNoCheckList() {
        noCheckList.add("com.google");
        noCheckList.add("io.netty");
    }

    /**
     * Checks a given {@link LinkedList<Class>} for OpModes. All OpModes found are then returned via
     * a HashMap containing the proper groupings.
     *
     * @param klazzes a LinkedList to check for possible OpMode candidates
     * @return a HashMap containing OpMode groupings
     */
    private HashMap<String, LinkedList<Class<OpMode>>> findOpModes(LinkedList<Class> klazzes) {
        Class<RedBlueAutonomous> autoClass = RedBlueAutonomous.class;
        Class<Disabled> disabledClass = Disabled.class;

        int nextAvailableIndex = 0;
        HashMap<String, LinkedList<Class<OpMode>>> opModes = new HashMap<>();
        for (Class currentClass : klazzes) {
            /*if (!currentClass.isAssignableFrom(OpMode.class)) {
                continue;
            }*/

            if (!currentClass.isAnnotationPresent(disabledClass)) {
                if (currentClass.isAnnotationPresent(autoClass)) {
                    Annotation annotation = currentClass.getAnnotation(autoClass);
                    String name = ((RedBlueAutonomous) annotation).name();
                    nextAvailableIndex = getNextAvailableIndex(nextAvailableIndex, opModes, currentClass, name);
                }
            }
        }
        return opModes;
    }

    /**
     * This generates an Application Context to use for getting the class package loader.
     *
     * @return an new Application Context, this Context will throw a {@link ClassCastException} if
     * it is cast to an Activity
     */
    private Context getApplicationContext() {
        final Class<?> activityThreadClass;
        Context ctx = null;

        try {
            activityThreadClass = Class.forName("android.app.ActivityThread");

            final Method method = activityThreadClass.getMethod("currentApplication");
            ctx = (Application) method.invoke(null, (Object[]) null);
        } catch (ClassNotFoundException | IllegalAccessException |
                NoSuchMethodException | InvocationTargetException e) {
            Log.wtf(TAG, e);
        }

        return ctx;
    }

    /**
     * Gets the next possible grouping in the HashMap and handles it, also handling the incremation
     * of the state index
     */
    private int getNextAvailableIndex(int nextAvailableIndex, HashMap<String, LinkedList<Class<OpMode>>> opModes, Class<OpMode> currentClass, String name) {
        if (name.equals("")) {
            int i = ++nextAvailableIndex;
            LinkedList<Class<OpMode>> temp = new LinkedList<>();
            temp.add(currentClass);
            opModes.put(Integer.toString(i), temp);
        } else {
            if (opModes.containsKey(name)) {
                opModes.get(name).add(currentClass);
            } else {
                LinkedList<Class<OpMode>> temp = new LinkedList<>();
                temp.add(currentClass);
                opModes.put(name, temp);
            }
        }
        return nextAvailableIndex;
    }

    /**
     * Checks the given class against the no-check list, returning {@code true} if the Class is not
     * within the list generated by {@link #buildNoCheckList()}
     *
     * @param klazz the name of the class to check
     * @return {@code true} if the class is not within the no-check-list; otherwise {@code true}
     */
    private boolean shouldAdd(String klazz) {
        // Check against every class name in the no-add list
        for (String noContinue : noCheckList) {
            if (klazz.contains(noContinue)) {
                return false;
            }
        }
        return true;
    }

    /**
     * This sorts the {@link LinkedList<Class>} sorting the OpModes given by a special character
     *
     * @param opModes the OpMode HashMap
     */
    private void sortOpModeMap(HashMap<String, LinkedList<Class<OpMode>>> opModes) {
        Comparator<Class> firstComparator =
                new OpModeComparator();

        for (String key : opModes.keySet()) {
            Collections.sort(opModes.get(key), firstComparator);
        }
    }

    /**
     * Converts the OpMode HashMap to a TreeMap, using the proper sorting algorithm.
     *
     * @param opModes the {@code HashMap} containing the properly grouped OpModes
     * @return a {@link TreeMap} with a proper sorting
     */
    private TreeMap<String, LinkedList<Class<OpMode>>> treeMapify(
            HashMap<String, LinkedList<Class<OpMode>>> opModes) {

        TreeMap<String, LinkedList<Class<OpMode>>> sortedOpModes = new TreeMap<>();
        for (String key : opModes.keySet()) {
            Class<OpMode> opMode = opModes.get(key).getFirst();
            String name = getOpModeName(opMode);

            sortedOpModes.put(name, opModes.get(key));
        }

        return sortedOpModes;
    }

    private static class OpModeComparator implements Comparator<Class> {
        @Override
        public int compare(Class lhs, Class rhs) {
            if (lhs.isAnnotationPresent(RedBlueAutonomous.class) &&
                    rhs.isAnnotationPresent(RedBlueAutonomous.class)) {
                String lName = ((RedBlueAutonomous) lhs.getAnnotation(RedBlueAutonomous.class)).name();
                String rName = ((RedBlueAutonomous) rhs.getAnnotation(RedBlueAutonomous.class)).name();

                if (lName.equals("")) {
                    lName = lhs.getSimpleName();
                }

                if (rName.equals("")) {
                    rName = rhs.getSimpleName();
                }
                return lName.compareTo(rName);
            }

            return -1;
        }
    }
}