package org.team9432

import kotlinx.coroutines.*
import org.team9432.lib.coroutines.RobotScope
import org.team9432.lib.resource.Action
import java.util.*

object RobotController {
    private val coroutineScope = RobotScope

    private var currentDriverRequest: Action? = null

    private var queue: Queue<Action> = LinkedList()

    private var currentJob: Job? = null
    private var currentJobIsDriverRequest = false

    private fun executeRequest(action: Action, isDriverRequest: Boolean) {
        val conflictingJob = currentJob
        currentJob = coroutineScope.launch {
            try {
                currentJobIsDriverRequest = isDriverRequest

                // Cancel conflicting job
                withContext(NonCancellable) {
                    conflictingJob?.cancelAndJoin()
                }

                // run the action
                coroutineScope { action.invoke(this) }
            } finally {
                currentJob = null
                if (isDriverRequest) currentDriverRequest = null
                updateTasks()
            }
        }
    }

    fun setDriverRequest(action: Action) {
        currentDriverRequest = action
        updateTasks()
    }

    suspend fun joinCurrent() = currentJob?.join()

    fun clearDriverRequest() {
        if (currentJobIsDriverRequest) currentJob?.cancel()
    }

    fun queueRobotRequest(action: Action) = queue.add(action)


    fun updateTasks() {
        if (queue.isNotEmpty() && currentJob == null) {
            executeRequest(queue.poll(), isDriverRequest = false)
        } else if (currentDriverRequest != null && currentJob == null) {
            currentDriverRequest?.let { executeRequest(it, isDriverRequest = true) }
        } else if (currentDriverRequest != null && currentJobIsDriverRequest) {
            currentDriverRequest?.let { executeRequest(it, isDriverRequest = true) }
        }
    }
}