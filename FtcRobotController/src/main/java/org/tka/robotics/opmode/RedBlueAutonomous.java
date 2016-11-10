package org.tka.robotics.opmode;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@Retention(RetentionPolicy.RUNTIME)
public @interface RedBlueAutonomous {

    String name();

    String group() default "";
}
