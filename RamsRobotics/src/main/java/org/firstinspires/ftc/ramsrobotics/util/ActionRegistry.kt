package org.firstinspires.ftc.ramsrobotics.util

import org.firstinspires.ftc.ramsrobotics.actions.Action
import org.firstinspires.ftc.ramsrobotics.actions.ActionType
import java.util.function.Supplier

class ActionRegistry {
    private val actions = HashMap<ActionType, Supplier<Action>>()

    fun createNew(type: ActionType): Action? {
        return Optionull.map(
            actions.get(type)
        ) { obj -> obj!!.get() }
    }

    fun register(type: ActionType, action: Supplier<Action>) {
        actions.put(type, action)
    }
}
