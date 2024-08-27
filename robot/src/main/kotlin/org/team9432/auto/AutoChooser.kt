package org.team9432.auto

import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.auto.types.Auto
import org.team9432.auto.types.AutoType
import org.team9432.lib.coroutines.RobotScope
import org.team9432.lib.dashboard.AutoSelector
import kotlin.time.Duration.Companion.seconds

object AutoChooser {
    private const val CHOOSER_COUNT = 5
    private val choosers = List(CHOOSER_COUNT) { AutoSelector.DashboardQuestion("Option $it Chooser", "Option $it Question") }.toSet()

    private var currentlySelectedAuto: Auto? = null

    private val chooser = AutoSelector(choosers) {
        addQuestion("Which Auto?", { currentlySelectedAuto = it }) {
            AutoType.FourNote.addOptionToSelector(this)
        }
    }

    fun getAuto() = currentlySelectedAuto

    init {
        RobotScope.launch {
            while (true) {
                chooser.update()

                delay(0.25.seconds)
            }
        }
    }
}