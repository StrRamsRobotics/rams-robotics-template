package org.firstinspires.ftc.ramsrobotics

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.ramsrobotics.components.Component
import org.firstinspires.ftc.ramsrobotics.util.Alliance
import org.firstinspires.ftc.robotcore.external.Telemetry

class Robot(val alliance: Alliance?, hardwareMap: HardwareMap?, telemetry: Telemetry) {
    val telemetry = MultipleTelemetry(
        telemetry,
        FtcDashboard.getInstance().telemetry,
//        PanelsTelemetry.ftcTelemetry
    )
    val components = listOf<Component>(
    )

    /** robot pose cache */
    var pose: Pose? = null
    var turntablePosCache: Int? = null

    companion object {
        private var robot: Robot? = null

        val alliance: Alliance?
            get() = robot?.alliance

        val telemetry: Telemetry
            get() = robot?.telemetry ?: throw AssertionError("Robot must be initialized!")

        var pose: Pose
            get() = robot?.pose ?: Pose()
            set(value) {
                robot?.pose = value
            }

        val components: List<Component>
            get() = robot!!.components

        fun init(alliance: Alliance?, hardwareMap: HardwareMap?, telemetry: Telemetry) {
            robot = Robot(alliance, hardwareMap, telemetry)
        }
    }
}
