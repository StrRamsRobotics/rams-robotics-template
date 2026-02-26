package org.firstinspires.ftc.ramsrobotics.opmodes

import org.firstinspires.ftc.ramsrobotics.Robot

abstract class DebugOpMode : BaseOpMode() {
    /**
     * doesn't init anything in case of missing motors
     */
    public override fun initRobot() {
        Robot.init(null, null, telemetry)
    }

    override fun initialize() {
    }
}
