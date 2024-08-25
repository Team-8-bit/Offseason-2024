package org.team9432.auto

import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.auto.types.AutoType.FourNote.Companion.applyFourNoteSelectorOptions
import org.team9432.lib.coroutines.RobotScope
import org.team9432.lib.dashboard.AutoSelector
import kotlin.time.Duration.Companion.seconds

object AutoChooser {
    private const val CHOOSER_COUNT = 5
    private val choosers = List(CHOOSER_COUNT) { AutoSelector.DashboardQuestion("Option $it Chooser", "Option $it Question") }.toSet()

    private val chooser = AutoSelector(choosers) {
        addQuestion("Which Auto?") {
            addOption("<NA>")

            addOption("Four Note") {
                applyFourNoteSelectorOptions()
            }
        }
    }

    init {
        RobotScope.launch {
            while (true) {
                chooser.update()
                delay(0.25.seconds)
            }
        }
    }
}