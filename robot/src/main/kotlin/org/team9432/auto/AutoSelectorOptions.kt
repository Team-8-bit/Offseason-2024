package org.team9432.auto

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.util.*

class AutoSelectorOptions private constructor(val buildQuestions: AutoSelectorOptions.() -> Unit) {
    companion object {
        fun build(buildQuestions: AutoSelectorOptions.() -> Unit) = AutoSelectorOptions(buildQuestions)
    }

    var questions = mutableListOf<AutoSelectorQuestion>()

    fun update(choosers: Queue<AutoChooser.ChooserPair>) {
        questions.clear()
        buildQuestions()

        questions.forEach { it.update(choosers) }
    }

    fun addQuestion(question: String, addOptions: AutoSelectorQuestion.() -> Unit) {
        questions.add(AutoSelectorQuestion(question).apply { addOptions() })
    }

    class AutoSelectorQuestion internal constructor(val question: String) {
        val options = mutableMapOf<String, AutoSelectorOptions?>()

        internal fun update(choosers: Queue<AutoChooser.ChooserPair>) {
            val targetChooser = choosers.poll()
            val newAnswer = targetChooser.chooser.get()
            SmartDashboard.putString(targetChooser.questionKey, question)
            targetChooser.chooser.setOptions(options.keys.toTypedArray())

            options.get(newAnswer)?.update(choosers)
        }

        fun addOption(answer: String, buildQuestions: AutoSelectorOptions.() -> Unit) {
            val update = AutoSelectorOptions(buildQuestions)
            options.put(answer, update)
        }
    }
}