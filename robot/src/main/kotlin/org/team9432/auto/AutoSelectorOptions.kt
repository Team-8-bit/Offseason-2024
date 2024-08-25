package org.team9432.auto

import java.util.*


/**
 * Class for selecting different auto configurations.
 *
 */
class AutoSelectorOptions private constructor() {
    companion object {
        fun build(buildQuestions: AutoSelectorOptions.() -> Unit) = AutoSelectorOptions().apply(buildQuestions)
    }

    private val questions = mutableListOf<AutoSelectorQuestion>()

    fun update(choosers: Queue<AutoChooser.ChooserPair>) {
        questions.forEach { it.update(choosers) }
    }

    fun addQuestion(question: String, addOptions: AutoSelectorQuestion.() -> Unit) {
        questions.add(AutoSelectorQuestion(question).apply(addOptions))
    }

    class AutoSelectorQuestion internal constructor(private val question: String) {
        private val options = mutableMapOf<String, AutoSelectorOptions?>()

        internal fun update(choosers: Queue<AutoChooser.ChooserPair>) {
            val targetChooser = choosers.poll() // Take the first chooser in the list
            targetChooser.update(question, options.keys) // Display question

            // Update the nested questions
            val answer = targetChooser.chooser.get()
            options[answer]?.update(choosers)
        }

        fun addOption(answer: String, buildQuestions: (AutoSelectorOptions.() -> Unit)? = null) {
            if (buildQuestions != null) {
                options[answer] = AutoSelectorOptions().apply(buildQuestions)
            } else {
                options[answer] = null
            }
        }
    }

}