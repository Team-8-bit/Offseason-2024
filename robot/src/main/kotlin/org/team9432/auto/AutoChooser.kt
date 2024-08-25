package org.team9432.auto

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.auto.types.AutoType.FourNote.Companion.applyFourNoteSelectorOptions
import org.team9432.lib.SwitchableChooser
import org.team9432.lib.coroutines.RobotScope
import java.util.*
import kotlin.time.Duration.Companion.seconds

object AutoChooser {
    data class ChooserPair(val chooser: SwitchableChooser, val questionKey: String) {
        fun reset() = update("", emptySet())

        fun update(question: String, options: Collection<String>) {
            SmartDashboard.putString(questionKey, question)
            chooser.setOptions(options.toTypedArray())
        }
    }

    const val CHOOSER_COUNT = 5

    private val choosers = List(CHOOSER_COUNT) { ChooserPair(SwitchableChooser("Option $it Chooser"), "Option $it Question") }

    init {
        RobotScope.launch {
            while (true) {
                update()
                delay(0.25.seconds)
            }
        }
    }

    val chooser = AutoSelectorOptions.build {
        addQuestion("Which Auto?") {
            addOption("<NA>")

            addOption("Four Note") {
                applyFourNoteSelectorOptions()
            }
        }
    }

    fun update() {
        val chooquoo: Queue<ChooserPair> = LinkedList(choosers)
        chooser.update(chooquoo)
        chooquoo.toList().forEach { it.reset() }
    }
}