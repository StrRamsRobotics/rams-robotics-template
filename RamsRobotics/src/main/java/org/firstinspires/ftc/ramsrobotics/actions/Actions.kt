package org.firstinspires.ftc.ramsrobotics.actions

import org.firstinspires.ftc.ramsrobotics.util.ActionRegistry
import java.util.function.Supplier

object Actions {
    val ACTIONS = ActionRegistry()

    private fun register(type: ActionType, action: Supplier<Action>) {
        ACTIONS.register(type, action)
    }

    init {
        register(ActionType.EXAMPLE) {
            InSequence(
                InParallel(
                    Callback {
                    }
                ),
                Callback {
                }
            )
        }
    }
}
