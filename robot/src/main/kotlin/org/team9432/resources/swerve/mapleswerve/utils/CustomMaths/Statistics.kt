package org.team9432.resources.swerve.mapleswerve.utils.CustomMaths

import kotlin.math.sqrt

object Statistics {
    fun getMean(dataSet: DoubleArray): Double {
        var sum: Double = 0.0
        for (data: Double in dataSet) sum += data
        return sum / dataSet.size
    }

    fun getMedian(dataSet: DoubleArray): Double {
        if (dataSet.size % 2 == 0) return (dataSet.get(dataSet.size / 2 - 1) + dataSet.get(dataSet.size / 2)) / 2

        return dataSet.get(dataSet.size / 2)
    }

    fun getStandardDeviation(dataSet: DoubleArray): Double {
        val mean: Double = getMean(dataSet)
        var varianceSum: Double = 0.0
        for (data: Double in dataSet) varianceSum += (data - mean) * (data - mean)
        return sqrt(varianceSum / (dataSet.size - 1))
    }

    fun getStandardizedScores(dataSet: DoubleArray): DoubleArray {
        require(dataSet.size > 1) { "data too short!!!" }
        val standardizedScores: DoubleArray = DoubleArray(dataSet.size)
        val mean: Double = getMean(dataSet)
        val standardDeviation: Double = getStandardDeviation(dataSet)
        for (i in dataSet.indices) standardizedScores.set(i, (dataSet.get(i) - mean) / standardDeviation)
        return standardizedScores
    }

    fun getCorrelationCoefficient(dataSet1: DoubleArray, dataSet2: DoubleArray): Double {
        require(dataSet1.size == dataSet2.size) { "data set length unmatched" }
        val standardizedScores1: DoubleArray = getStandardizedScores(dataSet1)
        val standardizedScores2: DoubleArray = getStandardizedScores(dataSet2)
        var productSum: Double = 0.0
        for (i in dataSet2.indices) productSum += standardizedScores1.get(i) * standardizedScores2.get(i)
        return productSum / (dataSet1.size - 1)
    }

    fun getBestFitLineSlope(dataSetX: DoubleArray, dataSetY: DoubleArray): Double {
        val standardizedDeviationX: Double = getStandardDeviation(dataSetX)
        val standardizedDeviationY: Double = getStandardDeviation(dataSetY)
        return (getCorrelationCoefficient(dataSetX, dataSetY)
                * standardizedDeviationY
                / standardizedDeviationX)
    }

    fun getBestFitLineIntersect(dataSetX: DoubleArray, dataSetY: DoubleArray): Double {
        val slope: Double = getBestFitLineSlope(dataSetX, dataSetY)
        return getMean(dataSetY) - slope * getMean(dataSetX)
    }
}
