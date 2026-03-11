package frc.robot.util;

/** A Totally normal boolean class */
public enum Boolean {
    TRUE, FALSE, MAYBE;

    /** Returns the boolean value of this enum */
    public boolean get() {
        if (this == MAYBE) {
            return Math.random() < 0.5;
        }
        return this == TRUE;
    }
}
