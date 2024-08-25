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
    val typeChooser = SwitchableChooser("Auto Type")

    data class ChooserPair(val chooser: SwitchableChooser, val questionKey: String) {
        fun reset() {
            SmartDashboard.putString(questionKey, "")
            chooser.setOptions(emptyArray())
        }
    }

    val chooserCount = 5

    val choosers = List(chooserCount) { ChooserPair(SwitchableChooser("Option $it Chooser"), "Option $it Question") }

    init {
        typeChooser.setOptions(arrayOf("<NA>", "Four Note"))
        RobotScope.launch {
            while (true) {
                update()
                delay(0.25.seconds)
            }
        }
    }

    val chooser = AutoSelectorOptions.build {
        addQuestion("Which Auto?") {
            addOption("<NA>") {

            }

            addOption("Four Note") {
                applyFourNoteSelectorOptions()
            }
        }
    }

    fun update() {
        val chooquoo: Queue<ChooserPair> = LinkedList(choosers)
        chooser.update(chooquoo)
        chooquoo.toList().forEach { it.reset() }


//        when (typeChooser.get()) {
//            null -> choosers.forEach { it.reset() }
//
//            "Four Note" -> {
//                val chooquoo: Queue<ChooserPair> = LinkedList(choosers)
//                AutoType.FourNote.selectorOptions.update(chooquoo)
//                chooquoo.toList().forEach { it.reset() }
//
////                questions.questions.forEachIndexed { index, it ->
////                    val (chooser, questionKey) = choosers[index]
////
////                    SmartDashboard.putString(questionKey, it.question)
////                    chooser.setOptions(it.options.keys.toTypedArray())
////                }
//
//
////                AutoType.FourNote.getOptions().entries.forEachIndexed { index, entry ->
////                    val (chooser, questionKey) = choosers[index]
////
////                    SmartDashboard.putString(questionKey, entry.key)
////                    chooser.setOptions(entry.value.toTypedArray())
////                }
//            }
//        }
//        RobotScope.launch {
//            delay(5.seconds)
//
//            typeChooser.setOptions(arrayOf("ooh", "look it works"))
//            delay(4.seconds)
//            typeChooser.setOptions(arrayOf("this is cool", "look it works", "three"))
    }
}