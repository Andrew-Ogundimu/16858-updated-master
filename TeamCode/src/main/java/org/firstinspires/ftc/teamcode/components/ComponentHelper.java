package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.CommonVariables;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;

/**
 * This class is responsible for getting component instances.
 */
public class ComponentHelper {

    private static HashMap<Class<? extends Component>, Object> instances = new HashMap<Class<? extends Component>, Object>();

    public static <T extends Component> T getComponent(Class<T> clazz, CommonVariables commonVariables) {
        Object instance = instances.get(clazz);
        if (instance != null) {
            T castedInstance = (T) instance;

            if (castedInstance.opMode == commonVariables.getOpMode()) {
                return (T) instances.get(clazz);
            }
        }

        try {
            Constructor<T> ctor = clazz.getConstructor(CommonVariables.class);
            T createdInstance = ctor.newInstance(commonVariables);

            instances.put(clazz, createdInstance);

            return createdInstance;
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (InstantiationException e) {
            e.printStackTrace();
        } catch (InvocationTargetException e) {
            e.printStackTrace();
        }

        return null;
    }
}
