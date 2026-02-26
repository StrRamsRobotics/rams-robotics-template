package org.firstinspires.ftc.ramsrobotics.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.ramsrobotics.Robot
import org.firstinspires.ftc.ramsrobotics.actions.Action
import org.firstinspires.ftc.ramsrobotics.actions.ActionType
import org.firstinspires.ftc.ramsrobotics.actions.Actions
import org.firstinspires.ftc.ramsrobotics.actions.BaseAction
import org.firstinspires.ftc.ramsrobotics.components.Component
import org.firstinspires.ftc.ramsrobotics.util.Alliance
import org.firstinspires.ftc.ramsrobotics.util.GlobalVars
import org.firstinspires.ftc.ramsrobotics.util.Logger
import org.firstinspires.ftc.ramsrobotics.util.Optionull
import java.util.function.Consumer

abstract class BaseOpMode : OpMode() {
    protected val runningActions: HashMap<ActionType, Action> = hashMapOf()
    protected val driverPad: Gamepad = Gamepad()
    protected val ctrlPad: Gamepad = Gamepad()
    protected val prevDriverPad: Gamepad = Gamepad()
    protected val prevCtrlPad: Gamepad = Gamepad()
    private var loopError = false
    private var loopErrorSummary = ""

    protected open fun getAlliance(): Alliance? = null

    override fun init() {
        initRobot()
        initGamepads()
        initialize()
    }

    protected open fun initRobot() {
        Robot.init(getAlliance(), hardwareMap, telemetry)
    }

    protected fun initGamepads() {
        driverPad.copy(gamepad1)
        ctrlPad.copy(gamepad2)
    }

    protected open fun initialize() {
    }

    override fun loop() {
        if(loopError) {
            Logger.add("Fatal Loop Error", loopErrorSummary)
            update()
            return
        }
        try {
            tick()
            tickActions()
            Robot.components.forEach(Consumer { obj: Component? -> obj!!.tick() })
            logging()
            update()
        }
        catch(t: Throwable) {
            loopError = true
            val message = t.message
            if(message == null || message.isEmpty()) {
                loopErrorSummary = t.javaClass.getSimpleName()
            }
            else {
                loopErrorSummary = t.javaClass.getSimpleName() + ": " + message
            }
            RobotLog.ee("BaseOpMode", t, "Unhandled exception in %s", javaClass.getSimpleName())
            Logger.add("Fatal Loop Error", loopErrorSummary)
            val stackTrace = t.stackTrace
            if(stackTrace.size > 0) {
                Logger.add("At", stackTrace[0].toString())
            }
            stopAllActions()
            requestOpModeStop()
        }
    }

    /**
     * Code written here is called every tick
     */
    abstract fun tick()

    /**
     * Any logging that doesn't use a value from a specific location can be written here <br></br>
     * This includes driver instructions and other stuff that needs to be logged constantly
     */
    protected open fun logging() {
    }

    protected fun update() {
        prevDriverPad.copy(driverPad)
        prevCtrlPad.copy(ctrlPad)
        driverPad.copy(gamepad1)
        ctrlPad.copy(gamepad2)
        Logger.update()
    }

    /**
     * actions are not queued; instead, trying to start an action after that
     * action has already started will result in nothing happening, as only
     * one action of a specific type can run at a time.
     */
    fun startAction(type: ActionType) {
        if(GlobalVars.disable_actions) {
            return
        }
        if(runningActions.containsKey(type)) {
            return
        }
        val action = Actions.ACTIONS.createNew(type)
        if(action != null) {
            action.start()
            runningActions.put(type, action)
        }
    }

    /**
     * actions are not queued; instead, trying to start an action after that
     * action has already started will result in nothing happening, as only
     * one action of a specific type can run at a time.
     *
     * @param timeout in seconds
     */
    fun startActionWithTimeout(type: ActionType, timeout: Double) {
        if(GlobalVars.disable_actions) {
            return
        }
        if(runningActions.containsKey(type)) {
            return
        }
        val action = Actions.ACTIONS.createNew(type)
        if(action != null) {
            if(action is BaseAction) {
                action.timeout(timeout)
            }
            action.start()
            runningActions.put(type, action)
        }
    }

    fun stopAction(type: ActionType) {
        Optionull.ifPresent(
            runningActions.remove(type)
        ) { obj: Action? -> obj!!.stop() }
    }

    fun stopAllActions() {
        runningActions.forEach { (_: ActionType, action: Action?) -> action!!.stop() }
        runningActions.clear()
    }

    fun isRunning(action: ActionType): Boolean {
        return runningActions.containsKey(action)
    }

    private fun tickActions() {
        if(GlobalVars.disable_actions) {
            return
        }
        // if an action has stopped, remove it from the runningActions list
        runningActions.entries.removeIf { entry ->
            entry.value.tickOrStop().stopped()
        }
    }
}
