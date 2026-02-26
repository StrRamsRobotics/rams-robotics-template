package org.firstinspires.ftc.ramsrobotics.util

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.ramsrobotics.actions.ActionType
import org.firstinspires.ftc.ramsrobotics.opmodes.auto.BaseAuto
import java.util.function.Consumer

class AutonSequence private constructor(private val sequence: MutableList<Consumer<Follower>>) {
    fun tick(follower: Follower, pathState: Int) {
        if(pathState >= 0 && pathState < sequence.size) {
            sequence[pathState].accept(follower)
        }
    }

    class Builder(private val instance: BaseAuto) {
        private val sequence: MutableList<Consumer<Follower>> = ArrayList()

        init {
            instance.setPathState(0)
        }

        fun followPath(path: PathChain?): Builder {
            path!!
            add { follower: Follower? ->
                follower!!.followPath(path)
                instance.advancePathState()
            }
            add { follower: Follower? ->
                if(!follower!!.isBusy) {
                    instance.advancePathState()
                }
            }
            return this
        }

        fun runAction(actionType: ActionType, repeat: Int): Builder {
            for(i in 0..<repeat) {
                runAction(actionType)
            }
            return this
        }

        fun runAction(actionType: ActionType): Builder {
            startAction(actionType)
            holdAction(actionType)
            return this
        }

        fun runActionWithTimeout(
            actionType: ActionType,
            timeoutSeconds: Double,
            repeat: Int
        ): Builder {
            for(i in 0..<repeat) {
                runActionWithTimeout(actionType, timeoutSeconds)
            }
            return this
        }

        fun runActionWithTimeout(actionType: ActionType, timeoutSeconds: Double): Builder {
            startActionWithTimeout(actionType, timeoutSeconds)
            holdAction(actionType)
            return this
        }

        fun startAction(actionType: ActionType): Builder {
            add { follower: Follower? ->
                instance.startAction(actionType)
                instance.advancePathState()
            }
            return this
        }

        fun startActionWithTimeout(actionType: ActionType, timeoutSeconds: Double): Builder {
            add { follower: Follower? ->
                instance.startActionWithTimeout(actionType, timeoutSeconds)
                instance.advancePathState()
            }
            return this
        }

        fun holdAction(actionType: ActionType): Builder {
            add { follower: Follower? ->
                if(!instance.isRunning(actionType)) {
                    instance.advancePathState()
                }
            }
            return this
        }

        fun stopAction(actionType: ActionType): Builder {
            add { follower: Follower? ->
                instance.stopAction(actionType)
                instance.advancePathState()
            }
            return this
        }

        fun add(action: Consumer<Follower>): Builder {
            sequence.add(action)
            return this
        }

        fun build(): AutonSequence {
            add { follower: Follower -> instance.setEndPathState() }
            return AutonSequence(sequence)
        }
    }
}
