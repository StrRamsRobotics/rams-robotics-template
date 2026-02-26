package org.firstinspires.ftc.ramsrobotics.util

import org.firstinspires.ftc.ramsrobotics.Robot
import org.firstinspires.ftc.ramsrobotics.actions.Callback
import java.util.function.Consumer

enum class Alliance {
    BLUE,
    RED;

    companion object {
        /**
         * @return value based on the current alliance, or blue if alliance is null
         */
        fun <T> map(blue: T, red: T): T {
            return when(Robot.alliance) {
                BLUE -> blue
                RED -> red
                null -> blue
            }
        }

        fun <T> map(func: Consumer<T?>, blue: T?, red: T?) {
            when(Robot.alliance) {
                BLUE -> func.accept(blue)
                RED -> func.accept(red)
                null -> {}
            }
        }

        fun map(blue: Callback, red: Callback) {
            when(Robot.alliance) {
                BLUE -> blue.run()
                RED -> red.run()
                null -> {}
            }
        }
    }
}
