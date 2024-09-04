package org.team9432.auto

import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.Robot
import org.team9432.auto.types.AmpsideCenterline
import org.team9432.auto.types.Auto
import org.team9432.auto.types.FarsideCenterline
import org.team9432.auto.types.FourNote
import org.team9432.lib.dashboard.AutoSelector
import org.team9432.lib.util.ChoreoUtil
import kotlin.time.Duration.Companion.seconds

object AutoChooser {
    private const val CHOOSER_COUNT = 5
    private val choosers = List(CHOOSER_COUNT) { AutoSelector.DashboardQuestion("Option $it Chooser", "Option $it Question") }.toSet()

    private var currentlySelectedAuto: Auto? = null

    private val chooser = AutoSelector(choosers) {
        addQuestion("Which Auto?", { currentlySelectedAuto = it }) {
            FourNote.addOptionToSelector(this)
            FarsideCenterline.addOptionToSelector(this)
            AmpsideCenterline.addOptionToSelector(this)
        }
    }

    fun getAuto() = currentlySelectedAuto

    init {
        Robot.coroutineScope.launch {
            while (true) {
                chooser.update()

                if (Robot.isDisabled) {
                    getAuto()?.let { it.getTrajectoryNames().forEach { ChoreoUtil.getTrajectoryWithCache(it) } }
                }

                delay(0.25.seconds)
            }
        }
    }
}