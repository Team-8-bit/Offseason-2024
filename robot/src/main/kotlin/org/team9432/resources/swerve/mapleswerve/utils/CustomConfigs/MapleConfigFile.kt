// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/ and ChatGPT
package org.team9432.resources.swerve.mapleswerve.utils.CustomConfigs

import edu.wpi.first.wpilibj.Filesystem
import org.w3c.dom.Element
import org.w3c.dom.Node
import org.w3c.dom.NodeList
import java.io.File
import java.io.FileWriter
import java.io.IOException
import java.nio.file.Paths
import javax.xml.parsers.DocumentBuilderFactory

class MapleConfigFile(val configType: String, val configName: String) {
    class ConfigBlock(val blockName: String) {
        private val doubleConfigs: MutableMap<String, Double> = HashMap()
        private val intConfigs: MutableMap<String, Int> = HashMap()
        private val stringConfigs: MutableMap<String, String> = HashMap()

        val configOrders: MutableList<String> = ArrayList()

        fun hasDoubleConfig(name: String): Boolean {
            return doubleConfigs.containsKey(name)
        }

        fun hasIntConfig(name: String): Boolean {
            return intConfigs.containsKey(name)
        }

        fun hasStringConfig(name: String): Boolean {
            return stringConfigs.containsKey(name)
        }

        @Throws(NullPointerException::class) fun getDoubleConfig(name: String): Double {
            if (!hasDoubleConfig(name)) throw NullPointerException(
                "Configuration not found for block: $blockName, config: $name, type: double"
            )
            return doubleConfigs[name]!!
        }

        @Throws(NullPointerException::class) fun getIntConfig(name: String): Int {
            if (!hasIntConfig(name)) throw NullPointerException(
                "Configuration not found for block: $blockName, config: $name, type: int"
            )
            return intConfigs[name]!!
        }

        @Throws(NullPointerException::class) fun getStringConfig(name: String): String? {
            if (!hasStringConfig(name)) throw NullPointerException(
                "Configuration not found for block: $blockName, config: $name, type: string"
            )
            return stringConfigs[name]
        }

        @Throws(IllegalArgumentException::class) fun putDoubleConfig(configName: String, value: Double) {
            require(!(intConfigs.containsKey(configName) || stringConfigs.containsKey(configName))) {
                ("Cannot put double config '"
                        + configName
                        + "' to block '"
                        + blockName
                        + "' since there is already an int or string config with the same name")
            }
            configOrders.add(configName)
            doubleConfigs[configName] = value
        }

        @Throws(IllegalArgumentException::class) fun putIntConfig(configName: String, value: Int) {
            require(!(doubleConfigs.containsKey(configName) || stringConfigs.containsKey(configName))) {
                ("Cannot put int config '"
                        + configName
                        + "' to block '"
                        + blockName
                        + "' since there is already a double or string config with the same name")
            }
            configOrders.add(configName)
            intConfigs[configName] = value
        }

        @Throws(IllegalArgumentException::class) fun putStringConfig(configName: String, value: String) {
            require(!(doubleConfigs.containsKey(configName) || intConfigs.containsKey(configName))) {
                ("Cannot put string config '"
                        + configName
                        + "' to block '"
                        + blockName
                        + "' since there is already a double or int config with the same name")
            }
            configOrders.add(configName)
            stringConfigs[configName] = value
        }
    }

    private val configBlocks: MutableMap<String, ConfigBlock> = HashMap()
    private val configBlocksOrder: MutableList<String> = ArrayList()

    fun getBlock(blockName: String): ConfigBlock? {
        if (!configBlocksOrder.contains(blockName)) {
            configBlocks[blockName] = ConfigBlock(blockName)
            configBlocksOrder.add(blockName)
        }
        return configBlocks[blockName]
    }

    fun saveConfigToUSBSafe() {
        try {
            saveConfigToUSB()
        } catch (ignored: IOException) {
        }
    }

    @Throws(IOException::class) fun saveConfigToUSB() {
        val usbDir = File("/U/")
        if (!usbDir.exists()) {
            throw IOException("No USB connected")
        }
        val configDir = File(usbDir, "savedConfigs/" + this.configType)
        if (!configDir.exists() && !configDir.mkdirs()) {
            throw IOException("Failed to create config directory on USB")
        }
        val configFile = File(configDir, this.configName + ".xml")
        FileWriter(configFile).use { writer ->
            writer.write("<" + this.configType + ">\n")
            writeAllConfigBlocks(this, writer)
            writer.write("</" + this.configType + ">\n")
        }
    }

    companion object {
        @Throws(IllegalArgumentException::class, IOException::class) fun fromDeployedConfig(configType: String, configName: String): MapleConfigFile {
            val configFile = MapleConfigFile(configType, configName)

            val xmlFilePath =
                Paths.get(
                    Filesystem.getDeployDirectory().path, "configs", configType, "$configName.xml"
                )
            val xmlFile = xmlFilePath.toFile()
            if (!xmlFile.exists()) {
                throw IOException("Config file does not exist: $xmlFilePath")
            }

            try {
                val dbFactory = DocumentBuilderFactory.newInstance()
                val dBuilder = dbFactory.newDocumentBuilder()
                val doc = dBuilder.parse(xmlFile)
                doc.documentElement.normalize()

                require(doc.documentElement.nodeName == configType) { "Root element is not $configType" }

                val blocks = doc.documentElement.childNodes
                processBlocks(configFile, blocks)
            } catch (e: Exception) {
                throw IOException("Error reading config file", e)
            }

            return configFile
        }

        private fun processBlocks(configFile: MapleConfigFile, blocks: NodeList) {
            for (i in 0 until blocks.length) {
                val blockNode = blocks.item(i)
                if (blockNode.nodeType == Node.ELEMENT_NODE) {
                    val blockElement = blockNode as Element
                    val blockName = blockElement.tagName
                    val block = ConfigBlock(blockName)
                    readBlockConfig(blockElement, block)
                    configFile.configBlocks[blockName] = block
                    configFile.configBlocksOrder.add(blockName)
                }
            }
        }

        private fun readBlockConfig(blockElement: Element, block: ConfigBlock) {
            val configNodes = blockElement.childNodes
            for (j in 0 until configNodes.length) {
                val configNode = configNodes.item(j)
                if (configNode.nodeType == Node.ELEMENT_NODE) {
                    addConfigToBlock(configNode as Element, block)
                }
            }
        }

        private fun addConfigToBlock(configElement: Element, block: ConfigBlock) {
            val configTag = configElement.tagName
            val type = configElement.getAttribute("type")
            val value = configElement.textContent

            when (type) {
                "double" -> block.putDoubleConfig(configTag, value.toDouble())
                "int" -> block.putIntConfig(configTag, value.toInt())
                "string" -> block.putStringConfig(configTag, value)
            }
        }

        @Throws(IOException::class) private fun writeAllConfigBlocks(config: MapleConfigFile, writer: FileWriter) {
            for (blockName in config.configBlocksOrder) {
                val block = config.configBlocks[blockName]
                writer.write("    <" + block!!.blockName + ">\n")
                writeSingleConfigBlock(block, writer)
                writer.write("    </" + block.blockName + ">\n")
            }
        }

        @Throws(IOException::class) private fun writeSingleConfigBlock(block: ConfigBlock, writer: FileWriter) {
            for (configName in block.configOrders) {
                if (block.hasStringConfig(configName)) writeStringConfig(block, configName, writer)
                else if (block.hasIntConfig(configName)) writeIntConfig(block, configName, writer)
                else if (block.hasDoubleConfig(configName)) writeDoubleConfig(block, configName, writer)
            }
        }

        @Throws(IOException::class) private fun writeDoubleConfig(block: ConfigBlock, configName: String, writer: FileWriter) {
            writer.write(
                ("        <"
                        + configName
                        + " type=\"double\">"
                        + block.getDoubleConfig(configName)
                        + "</"
                        + configName
                        + ">\n")
            )
        }

        @Throws(IOException::class) private fun writeStringConfig(block: ConfigBlock, configName: String, writer: FileWriter) {
            writer.write(
                ("        <"
                        + configName
                        + " type=\"string\">"
                        + block.getStringConfig(configName)
                        + "</"
                        + configName
                        + ">\n")
            )
        }

        @Throws(IOException::class) private fun writeIntConfig(block: ConfigBlock, configName: String, writer: FileWriter) {
            writer.write(
                ("        <"
                        + configName
                        + " type=\"int\">"
                        + block.getIntConfig(configName)
                        + "</"
                        + configName
                        + ">\n")
            )
        }
    }
}
