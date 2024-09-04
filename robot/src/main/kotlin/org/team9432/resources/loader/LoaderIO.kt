package org.team9432.resources.loader

import org.team9432.annotation.Logged

interface LoaderIO {
    fun setVoltage(volts: Double)

    fun updateInputs(inputs: LoaderIOInputs)

    @Logged
    open class LoaderIOInputs {
        var motorSpeed: Double = 0.0
    }
}