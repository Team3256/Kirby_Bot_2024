package frc.robot.utils;

import edu.wpi.first.hal.HALUtil;

public class StackJumper {
    private static boolean traceTime = true;

    public static String getCallerMethodName() {
        if (traceTime) {
            double d = HALUtil.getFPGATime();
            String methodName = StackWalker.getInstance().walk(stream -> stream.skip(2).findFirst().get())
                    .getMethodName();
            System.out.println("Time to get method name: " + (HALUtil.getFPGATime() - d) + " ns for " + methodName);
            return methodName;
        } else {
            return StackWalker.getInstance().walk(stream -> stream.skip(2).findFirst().get()).getMethodName();
        }
    }
}
