#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License along with
#  this program; if not, write to the Free Software Foundation, Inc.,
#  51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

"""
The flight control tab shows telemetry data and flight settings.
"""

import logging

from PyQt5 import uic
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal, QAbstractItemModel, QModelIndex
from PyQt5.QtWidgets import QMessageBox, QLabel
from PyQt5.QtGui import QPixmap

import cfclient
from cfclient.ui.widgets.ai import AttitudeIndicator
from cfclient.ui.widgets.plotwidget import PlotWidget

from cfclient.utils.config import Config
from cflib.crazyflie.log import LogConfig

from cfclient.utils.input import JoystickReader

from cfclient.ui.tab import Tab

LOG_NAME_ESTIMATED_Z = "stateEstimate.z"

__author__ = 'Bitcraze AB'
__all__ = ['SwarmTab']

logger = logging.getLogger(__name__)

example_tab_class = uic.loadUiType(cfclient.module_path +
                                  "/ui/tabs/swarmTab.ui")[0]

MAX_THRUST = 65536.0


class LogConfigModel(QAbstractItemModel):
    """Model for log configurations in the ComboBox"""

    def __init__(self, parent=None):
        super(LogConfigModel, self).__init__(parent)
        self._nodes = []

    def add_block(self, block):
        self._nodes.append(block)
        self.layoutChanged.emit()

    def parent(self, index):
        """Re-implemented method to get the parent of the given index"""
        return QModelIndex()

    def remove_block(self, block):
        """Remove a block from the view"""
        raise NotImplementedError()

    def columnCount(self, parent):
        """Re-implemented method to get the number of columns"""
        return 1

    def rowCount(self, parent):
        """Re-implemented method to get the number of rows for a given index"""
        parent_item = parent.internalPointer()
        if parent.isValid():
            parent_item = parent.internalPointer()  # noqa
            return 0
        else:
            return len(self._nodes)

    def index(self, row, column, parent):
        """Re-implemented method to get the index for a specified
        row/column/parent combination"""
        if not self._nodes:
            return QModelIndex()
        node = parent.internalPointer()
        if not node:
            index = self.createIndex(row, column, self._nodes[row])
            return index
        else:
            return self.createIndex(row, column, node.get_child(row))

    def data(self, index, role):
        """Re-implemented method to get the data for a given index and role"""
        node = index.internalPointer()  # noqa
        if not index.isValid() or not 0 <= index.row() < len(self._nodes):
            return None
        if role == Qt.DisplayRole:
            return self._nodes[index.row()].name
        return None

    def reset(self):
        """Reset the model"""
        self._nodes = []
        self.layoutChanged.emit()

    def get_config(self, i):
        return self._nodes[i]


class SwarmTab(Tab, example_tab_class):
    uiSetupReadySignal = pyqtSignal()

    _motor_data_signal = pyqtSignal(int, object, object)
    _imu_data_signal = pyqtSignal(int, object, object)
    _baro_data_signal = pyqtSignal(int, object, object)

    _input_updated_signal = pyqtSignal(float, float, float, float)
    _rp_trim_updated_signal = pyqtSignal(float, float)
    _emergency_stop_updated_signal = pyqtSignal(bool)
    _assisted_control_updated_signal = pyqtSignal(bool)
    _heighthold_input_updated_signal = pyqtSignal(float, float, float, float)
    _hover_input_updated_signal = pyqtSignal(float, float, float, float)

    _log_error_signal = pyqtSignal(object, str)

    _plotter_log_error_signal = pyqtSignal(object, str)
    _log_data_signal = pyqtSignal(int, object, object)
    _disconnected_signal = pyqtSignal(str)
    _connected_signal = pyqtSignal(str)

    colors = [
        (60, 200, 60),    # green
        (40, 100, 255),   # blue
        (255, 130, 240),  # magenta
        (255, 26, 28),    # red
        (255, 170, 0),    # orange
        (40, 180, 240),   # cyan
        (153, 153, 153),  # grey
        (176, 96, 50),    # brown
        (180, 60, 240),   # purple
    ]

    # UI_DATA_UPDATE_FPS = 10

    connectionFinishedSignal = pyqtSignal(str)
    disconnectedSignal = pyqtSignal(str)

    _limiting_updated = pyqtSignal(bool, bool, bool)

    def __init__(self, tabWidget, helper, *args):
        super(SwarmTab, self).__init__(*args)
        self.setupUi(self)

        self.tabName = "Swarm"
        self.menuName = "Swarm"

        self.tabWidget = tabWidget
        self.helper = helper

        self.disconnectedSignal.connect(self.disconnected)
        self.connectionFinishedSignal.connect(self.connected)
        # Incoming signals
        self.helper.cf.connected.add_callback(
            self.connectionFinishedSignal.emit)
        self.helper.cf.disconnected.add_callback(self.disconnectedSignal.emit)

        self._input_updated_signal.connect(self.updateInputControl)
        self.helper.inputDeviceReader.input_updated.add_callback(
            self._input_updated_signal.emit)
        self._rp_trim_updated_signal.connect(self.calUpdateFromInput)
        self.helper.inputDeviceReader.rp_trim_updated.add_callback(
            self._rp_trim_updated_signal.emit)
        self._emergency_stop_updated_signal.connect(self.updateEmergencyStop)
        self.helper.inputDeviceReader.emergency_stop_updated.add_callback(
            self._emergency_stop_updated_signal.emit)

        self.helper.inputDeviceReader.heighthold_input_updated.add_callback(
            self._heighthold_input_updated_signal.emit)
        self._heighthold_input_updated_signal.connect(
            self._heighthold_input_updated)
        self.helper.inputDeviceReader.hover_input_updated.add_callback(
            self._hover_input_updated_signal.emit)
        self._hover_input_updated_signal.connect(
            self._hover_input_updated)

        self.helper.inputDeviceReader.assisted_control_updated.add_callback(
            self._assisted_control_updated_signal.emit)

        self._assisted_control_updated_signal.connect(
            self._assisted_control_updated)

        self._imu_data_signal.connect(self._imu_data_received)
        self._baro_data_signal.connect(self._baro_data_received)
        self._motor_data_signal.connect(self._motor_data_received)

        self._log_error_signal.connect(self._logging_error)

        # PlotWidget Stuff
        self._plot = PlotWidget(fps=30)
        # Check if we could find the PyQtImport. If not, then
        # set this tab as disabled
        self.enabled = self._plot.can_enable
        self._model = LogConfigModel()
        self.dataSelector.setModel(self._model)
        self.plotLayout.addWidget(self._plot)
        self._log_data_signal.connect(self._log_data_received)
        self._plotter_log_error_signal.connect(self._plotter_logging_error)

        if self.enabled:
            self._disconnected_signal.connect(self._disconnected)
            self.helper.cf.disconnected.add_callback(
                self._disconnected_signal.emit)

            self._connected_signal.connect(self._connected)
            self.helper.cf.connected.add_callback(
                self._connected_signal.emit)

            self.helper.cf.log.block_added_cb.add_callback(self._config_added)
            self.dataSelector.currentIndexChanged.connect(
                self._selection_changed)

        self._previous_config = None
        self._started_previous = False

        # Connect UI signals that are in this tab
        #self.flightModeCombo.currentIndexChanged.connect(self.flightmodeChange)
        # self.minThrust.valueChanged.connect(self.minMaxThrustChanged)
        # self.maxThrust.valueChanged.connect(self.minMaxThrustChanged)
        # self.thrustLoweringSlewRateLimit.valueChanged.connect(
        #     self.thrustLoweringSlewRateLimitChanged)
        # self.slewEnableLimit.valueChanged.connect(
        #     self.thrustLoweringSlewRateLimitChanged)
        self.targetCalRoll.valueChanged.connect(self._trim_roll_changed)
        self.inputTypeComboBox.currentIndexChanged.connect(self.inputtypeChange)
        self.stepHeightCombo.valueChanged.connect(self._step_height_changed)
        self.frequencyCombo.valueChanged.connect(self._sine_frequency_changed)
        self.referenceHeightCheckbox.toggled.connect(lambda: self._use_ref_input(self.referenceHeightCheckbox.isChecked()))
        self.targetCalPitch.valueChanged.connect(self._trim_pitch_changed)
        self.updateGainsBtn.clicked.connect(self._controller_gain_changed)
        self.k1Combo.valueChanged.connect(self._k123_gain_changed)
        self.k2Combo.valueChanged.connect(self._k123_gain_changed)
        self.k3Combo.valueChanged.connect(self._k123_gain_changed)
        # self.maxAngle.valueChanged.connect(self.maxAngleChanged)
        # self.maxYawRate.valueChanged.connect(self.maxYawRateChanged)
        self.uiSetupReadySignal.connect(self.uiSetupReady)
        self.clientXModeCheckbox.toggled.connect(self.changeXmode)
        self.isInCrazyFlightmode = False
        self.uiSetupReady()

        # self.clientXModeCheckbox.setChecked(Config().get("client_side_xmode"))
        #
        # self.crazyflieXModeCheckbox.clicked.connect(
        #     lambda enabled:
        #     self.helper.cf.param.set_value("flightmode.x",
        #                                    str(enabled)))
        # self.helper.cf.param.add_update_callback(
        #     group="flightmode", name="xmode",
        #     cb=(lambda name, checked:
        #         self.crazyflieXModeCheckbox.setChecked(eval(checked))))
        #
        # self.ratePidRadioButton.clicked.connect(
        #     lambda enabled:
        #     self.helper.cf.param.set_value("flightmode.ratepid",
        #                                    str(enabled)))
        #
        # self.angularPidRadioButton.clicked.connect(
        #     lambda enabled:
        #     self.helper.cf.param.set_value("flightmode.ratepid",
        #                                    str(not enabled)))
        #
        # self._led_ring_headlight.clicked.connect(
        #     lambda enabled:
        #     self.helper.cf.param.set_value("ring.headlightEnable",
        #                                    str(enabled)))

        # self.helper.cf.param.add_update_callback(
        #     group="flightmode", name="ratepid",
        #     cb=(lambda name, checked:
        #         self.ratePidRadioButton.setChecked(eval(checked))))

        # self.helper.cf.param.add_update_callback(
        #     group="cpu", name="flash",
        #     cb=self._set_enable_client_xmode)

        # self.helper.cf.param.add_update_callback(
        #     group="ring", name="headlightEnable",
        #     cb=(lambda name, checked:
        #         self._led_ring_headlight.setChecked(eval(checked))))

        self._ledring_nbr_effects = 0

        # self.helper.cf.param.add_update_callback(
        #     group="ring",
        #     name="effect",
        #     cb=self._ring_effect_updated)

        self.helper.cf.param.add_update_callback(
            group="imu_sensors",
            cb=self._set_available_sensors)

        # self.helper.cf.param.all_updated.add_callback(
        #     self._ring_populate_dropdown)

        self.logBaro = None
        self.logAltHold = None

        self.ai = AttitudeIndicator()
        self.verticalLayout_2.addWidget(self.ai)
        # self.logo = QLabel(self)
        # pixmap = QPixmap('C:\dev\crazyflie-clients-python\src\cfclient/SUTDLogo.png')
        # self.logo.setPixmap(pixmap.scaled(150, 150, Qt.KeepAspectRatio))
        # self.verticalLayout.addWidget(self.logo)
        self.splitter.setSizes([1000, 1])

        self.targetCalPitch.setValue(Config().get("trim_pitch"))
        self.targetCalRoll.setValue(Config().get("trim_roll"))

        self.helper.inputDeviceReader.alt1_updated.add_callback(
            self.alt1_updated)
        self.helper.inputDeviceReader.alt2_updated.add_callback(
            self.alt2_updated)
        self._tf_state = 0
        self._ring_effect = 0

        # # Connect callbacks for input device limiting of rpöö/pitch/yaw/thust
        # self.helper.inputDeviceReader.limiting_updated.add_callback(
        #     self._limiting_updated.emit)
        # self._limiting_updated.connect(self._set_limiting_enabled)

    def _set_enable_client_xmode(self, name, value):
        if eval(value) <= 128:
            self.clientXModeCheckbox.setEnabled(True)
        else:
            self.clientXModeCheckbox.setEnabled(False)
            self.clientXModeCheckbox.setChecked(False)

    def _set_limiting_enabled(self, rp_limiting_enabled,
                              yaw_limiting_enabled,
                              thrust_limiting_enabled):
        self.maxAngle.setEnabled(rp_limiting_enabled)
        self.targetCalRoll.setEnabled(rp_limiting_enabled)
        self.targetCalPitch.setEnabled(rp_limiting_enabled)
        self.maxYawRate.setEnabled(yaw_limiting_enabled)
        self.maxThrust.setEnabled(thrust_limiting_enabled)
        self.minThrust.setEnabled(thrust_limiting_enabled)
        self.slewEnableLimit.setEnabled(thrust_limiting_enabled)
        self.thrustLoweringSlewRateLimit.setEnabled(thrust_limiting_enabled)

    def thrustToPercentage(self, thrust):
        return ((thrust / MAX_THRUST) * 100.0)

    def uiSetupReady(self):
        flightComboIndex = self.flightModeCombo.findText(
            Config().get("flightmode"), Qt.MatchFixedString)
        if (flightComboIndex < 0):
            self.flightModeCombo.setCurrentIndex(0)
            self.flightModeCombo.currentIndexChanged.emit(0)
        else:
            self.flightModeCombo.setCurrentIndex(flightComboIndex)
            self.flightModeCombo.currentIndexChanged.emit(flightComboIndex)

    def _logging_error(self, log_conf, msg):
        QMessageBox.about(self, "Log error",
                          "Error when starting log config [%s]: %s" % (
                              log_conf.name, msg))

    def _motor_data_received(self, timestamp, data, logconf):
        if self.isVisible():
            self.actualM1.setValue(data["motor.m1"])
            self.actualM2.setValue(data["motor.m2"])
            self.actualM3.setValue(data["motor.m3"])
            self.actualM4.setValue(data["motor.m4"])

    def _baro_data_received(self, timestamp, data, logconf):
        if self.isVisible():
            estimated_z = data[LOG_NAME_ESTIMATED_Z]
            self.actualHeight.setText(("%.2f" % estimated_z))
            self.ai.setBaro(estimated_z, self.is_visible())

    def _heighthold_input_updated(self, roll, pitch, yaw, height):
        if (self.isVisible() and
                (self.helper.inputDeviceReader.get_assisted_control() ==
                 self.helper.inputDeviceReader.ASSISTED_CONTROL_HEIGHTHOLD)):
            self.targetRoll.setText(("%0.2f deg" % roll))
            self.targetPitch.setText(("%0.2f deg" % pitch))
            self.targetYaw.setText(("%0.2f deg/s" % yaw))
            self.targetHeight.setText(("%.2f m" % height))
            self.ai.setHover(height, self.is_visible())

    def _hover_input_updated(self, vx, vy, yaw, height):
        if (self.isVisible() and
                (self.helper.inputDeviceReader.get_assisted_control() ==
                 self.helper.inputDeviceReader.ASSISTED_CONTROL_HOVER)):
            self.targetRoll.setText(("%0.2f m/s" % vy))
            self.targetPitch.setText(("%0.2f m/s" % vx))
            self.targetYaw.setText(("%0.2f deg/s" % yaw))
            self.targetHeight.setText(("%.2f m" % height))
            self.ai.setHover(height, self.is_visible())

    def _imu_data_received(self, timestamp, data, logconf):
        if self.isVisible():
            self.actualRoll.setText(("%.2f" % data["stabilizer.roll"]))
            self.actualPitch.setText(("%.2f" % data["stabilizer.pitch"]))
            self.actualYaw.setText(("%.2f" % data["stabilizer.yaw"]))
            self.actualThrust.setText("%.2f%%" %
                                      self.thrustToPercentage(
                                          data["stabilizer.thrust"]))

            self.ai.setRollPitch(-data["stabilizer.roll"],
                                 data["stabilizer.pitch"], self.is_visible())

    def connected(self, linkURI):
        # Reset Gains
        self.helper.cf.param.set_value("posCtlPid.zKd", str(0.0))
        self.helper.cf.param.set_value("posCtlPid.zKi", str(0.5))
        self.helper.cf.param.set_value("posCtlPid.zKp", str(40.0))
        self.k1Combo.setValue(40.0)
        self.helper.cf.param.set_value("posCtlPid.zVelMax", str(15.0))
        self.helper.cf.param.set_value("velCtlPid.vzKd", str(0.0))
        self.helper.cf.param.set_value("velCtlPid.vzKi", str(0.0))
        self.helper.cf.param.set_value("velCtlPid.vzKp", str(15.0))
        self.k2Combo.setValue(15.0)
        self.k3Combo.setValue(0.0)

        # IMU & THRUST
        lg = LogConfig("Stabilizer", Config().get("ui_update_period"))
        lg.add_variable("stabilizer.roll", "float")
        lg.add_variable("stabilizer.pitch", "float")
        lg.add_variable("stabilizer.yaw", "float")
        lg.add_variable("stabilizer.thrust", "uint16_t")

        try:
            self.helper.cf.log.add_config(lg)
            lg.data_received_cb.add_callback(self._imu_data_signal.emit)
            lg.error_cb.add_callback(self._log_error_signal.emit)
            lg.start()
        except KeyError as e:
            logger.warning(str(e))
        except AttributeError as e:
            logger.warning(str(e))

        # MOTOR
        lg = LogConfig("Motors", Config().get("ui_update_period"))
        lg.add_variable("motor.m1")
        lg.add_variable("motor.m2")
        lg.add_variable("motor.m3")
        lg.add_variable("motor.m4")

        try:
            self.helper.cf.log.add_config(lg)
            lg.data_received_cb.add_callback(self._motor_data_signal.emit)
            lg.error_cb.add_callback(self._log_error_signal.emit)
            lg.start()
        except KeyError as e:
            logger.warning(str(e))
        except AttributeError as e:
            logger.warning(str(e))

        self._populate_assisted_mode_dropdown()

    def _set_available_sensors(self, name, available):
        logger.info("[%s]: %s", name, available)
        available = eval(available)

        self.actualHeight.setEnabled(True)
        self.helper.inputDeviceReader.set_alt_hold_available(available)
        if not self.logBaro:
            # The sensor is available, set up the logging
            self.logBaro = LogConfig("Baro", 200)
            self.logBaro.add_variable(LOG_NAME_ESTIMATED_Z, "float")

            try:
                self.helper.cf.log.add_config(self.logBaro)
                self.logBaro.data_received_cb.add_callback(
                    self._baro_data_signal.emit)
                self.logBaro.error_cb.add_callback(
                    self._log_error_signal.emit)
                self.logBaro.start()
            except KeyError as e:
                logger.warning(str(e))
            except AttributeError as e:
                logger.warning(str(e))

    def disconnected(self, linkURI):
        self.ai.setRollPitch(0, 0)
        self.actualM1.setValue(0)
        self.actualM2.setValue(0)
        self.actualM3.setValue(0)
        self.actualM4.setValue(0)
        self.actualRoll.setText("")
        self.actualPitch.setText("")
        self.actualYaw.setText("")
        self.actualThrust.setText("")
        self.actualHeight.setText("")
        self.targetHeight.setText("Not Set")
        self.ai.setHover(0, self.is_visible())
        self.targetHeight.setEnabled(False)
        self.actualHeight.setEnabled(False)
        self.clientXModeCheckbox.setEnabled(False)
        self.logBaro = None
        self.logAltHold = None
        # self._led_ring_effect.setEnabled(False)
        # self._led_ring_effect.clear()
        # try:
        #     self._led_ring_effect.currentIndexChanged.disconnect(
        #         self._ring_effect_changed)
        # except TypeError:
        #     # Signal was not connected
        #     pass
        # self._led_ring_effect.setCurrentIndex(-1)
        # self._led_ring_headlight.setEnabled(False)

        try:
            self._assist_mode_combo.currentIndexChanged.disconnect(
                self._assist_mode_changed)
        except TypeError:
            # Signal was not connected
            pass
        self._assist_mode_combo.setEnabled(False)
        self._assist_mode_combo.clear()

    def minMaxThrustChanged(self):
        self.helper.inputDeviceReader.min_thrust = self.minThrust.value()
        self.helper.inputDeviceReader.max_thrust = self.maxThrust.value()
        if (self.isInCrazyFlightmode is True):
            Config().set("min_thrust", self.minThrust.value())
            Config().set("max_thrust", self.maxThrust.value())

    def thrustLoweringSlewRateLimitChanged(self):
        self.helper.inputDeviceReader.thrust_slew_rate = (
            self.thrustLoweringSlewRateLimit.value())
        self.helper.inputDeviceReader.thrust_slew_limit = (
            self.slewEnableLimit.value())
        if (self.isInCrazyFlightmode is True):
            Config().set("slew_limit", self.slewEnableLimit.value())
            Config().set("slew_rate", self.thrustLoweringSlewRateLimit.value())

    def maxYawRateChanged(self):
        logger.debug("MaxYawrate changed to %d", self.maxYawRate.value())
        self.helper.inputDeviceReader.max_yaw_rate = self.maxYawRate.value()
        if (self.isInCrazyFlightmode is True):
            Config().set("max_yaw", self.maxYawRate.value())

    def maxAngleChanged(self):
        logger.debug("MaxAngle changed to %d", self.maxAngle.value())
        self.helper.inputDeviceReader.max_rp_angle = self.maxAngle.value()
        if (self.isInCrazyFlightmode is True):
            Config().set("max_rp", self.maxAngle.value())

    def _trim_pitch_changed(self, value):
        logger.debug("Pitch trim updated to [%f]" % value)
        self.helper.inputDeviceReader.trim_pitch = value
        Config().set("trim_pitch", value)

    def _trim_roll_changed(self, value):
        logger.debug("Roll trim updated to [%f]" % value)
        self.helper.inputDeviceReader.trim_roll = value
        Config().set("trim_roll", value)

    def _k123_gain_changed(self):
        logger.debug("controller gains updated")
        self.updateGainsBtn.setStyleSheet("background-color:#0078d7; color:#ffffff;")
        self.resetGainsBtn.setStyleSheet("background-color:#0078d7; color:#ffffff;")

    def _controller_gain_changed(self):
        self.updateGainsBtn.setStyleSheet("background-color:#f0f0f0; color:#000000;")
        self.helper.cf.param.set_value("posCtlPid.zKi", str(self.k3Combo.value()))
        self.helper.cf.param.set_value("posCtlPid.zKp", str(self.k1Combo.value()))
        self.helper.cf.param.set_value("velCtlPid.vzKp", str(self.k2Combo.value()))

    def _reset_gain_changed(self):
        # Reset Gains
        self.k1Combo.value(40.0)
        self.k2Combo.setValue(15.0)
        self.k3Combo.setValue(0.0)
        self.helper.cf.param.set_value("posCtlPid.zKi", str(self.k3Combo.value()))
        self.helper.cf.param.set_value("posCtlPid.zKp", str(self.k1Combo.value()))
        self.helper.cf.param.set_value("velCtlPid.vzKp", str(self.k2Combo.value()))
        self.resetGainsBtn.setStyleSheet("background-color:#f0f0f0; color:#000000;")

    def _step_height_changed(self, reference):
        logger.debug("Reference height updated to [%f]" % reference)
        self.targetHeight.setText(("%.2f m" % reference))
        self.ai.setHover(reference, self.is_visible())
        self.helper.referenceHeight = reference
        self.helper.inputTime = 0.0

    def _sine_frequency_changed(self, freq):
        logger.debug("Sine-wave frequency updated to [%f]" % freq)
        self.helper.sinewaveFrequency = freq
        self.helper.inputTime = 0.0

    def _use_ref_input(self, enabled):
        logger.debug("Using reference input")
        self.helper.useReferenceHeight = enabled
        self.helper.inputTime = 0.0

    def inputtypeChange(self, item):
        logger.debug("Changed input type to %s",
                     self.inputTypeComboBox.itemText(item))
        self.helper.inputType = item
        self.helper.inputTime = 0.0
        # item is a number, #0 = 'Step Input', #1 = 'Sine Wave', #2 = 'Ramp'

    def calUpdateFromInput(self, rollCal, pitchCal):
        logger.debug("Trim changed on joystick: roll=%.2f, pitch=%.2f",
                     rollCal, pitchCal)
        self.targetCalRoll.setValue(rollCal)
        self.targetCalPitch.setValue(pitchCal)

    def updateInputControl(self, roll, pitch, yaw, thrust):
        self.targetRoll.setText(("%0.2f deg" % roll))
        self.targetPitch.setText(("%0.2f deg" % pitch))
        self.targetYaw.setText(("%0.2f deg/s" % yaw))
        self.targetThrust.setText(("%0.2f %%" %
                                   self.thrustToPercentage(thrust)))
        self.thrustProgress.setValue(thrust)

    def setMotorLabelsEnabled(self, enabled):
        self.M1label.setEnabled(enabled)
        self.M2label.setEnabled(enabled)
        self.M3label.setEnabled(enabled)
        self.M4label.setEnabled(enabled)

    def emergencyStopStringWithText(self, text):
        return ("<html><head/><body><p>"
                "<span style='font-weight:600; color:#7b0005;'>{}</span>"
                "</p></body></html>".format(text))

    def updateEmergencyStop(self, emergencyStop):
        if emergencyStop:
            self.setMotorLabelsEnabled(False)
            self.emergency_stop_label.setText(
                self.emergencyStopStringWithText("Kill switch active"))
        else:
            self.setMotorLabelsEnabled(True)
            self.emergency_stop_label.setText("")

    def flightmodeChange(self, item):
        Config().set("flightmode", str(self.flightModeCombo.itemText(item)))
        logger.debug("Changed flightmode to %s",
                     self.flightModeCombo.itemText(item))
        self.isInCrazyFlightmode = False
        if (item == 0):  # Normal
            self.maxAngle.setValue(Config().get("normal_max_rp"))
            self.maxThrust.setValue(Config().get("normal_max_thrust"))
            self.minThrust.setValue(Config().get("normal_min_thrust"))
            self.slewEnableLimit.setValue(Config().get("normal_slew_limit"))
            self.thrustLoweringSlewRateLimit.setValue(
                Config().get("normal_slew_rate"))
            self.maxYawRate.setValue(Config().get("normal_max_yaw"))
        if (item == 1):  # Advanced
            self.maxAngle.setValue(Config().get("max_rp"))
            self.maxThrust.setValue(Config().get("max_thrust"))
            self.minThrust.setValue(Config().get("min_thrust"))
            self.slewEnableLimit.setValue(Config().get("slew_limit"))
            self.thrustLoweringSlewRateLimit.setValue(
                Config().get("slew_rate"))
            self.maxYawRate.setValue(Config().get("max_yaw"))
            self.isInCrazyFlightmode = True

        if (item == 0):
            newState = False
        else:
            newState = True
        self.maxThrust.setEnabled(newState)
        self.maxAngle.setEnabled(newState)
        self.minThrust.setEnabled(newState)
        self.thrustLoweringSlewRateLimit.setEnabled(newState)
        self.slewEnableLimit.setEnabled(newState)
        self.maxYawRate.setEnabled(newState)

    def _assist_mode_changed(self, item):
        mode = None

        if (item == 0):  # Altitude hold
            mode = JoystickReader.ASSISTED_CONTROL_ALTHOLD
        if (item == 1):  # Position hold
            mode = JoystickReader.ASSISTED_CONTROL_POSHOLD
        if (item == 2):  # Position hold
            mode = JoystickReader.ASSISTED_CONTROL_HEIGHTHOLD
        if (item == 3):  # Position hold
            mode = JoystickReader.ASSISTED_CONTROL_HOVER

        self.helper.inputDeviceReader.set_assisted_control(mode)
        Config().set("assistedControl", mode)

    def _assisted_control_updated(self, enabled):
        if self.helper.inputDeviceReader.get_assisted_control() == \
                JoystickReader.ASSISTED_CONTROL_POSHOLD:
            self.targetThrust.setEnabled(not enabled)
            self.targetRoll.setEnabled(not enabled)
            self.targetPitch.setEnabled(not enabled)
        elif ((self.helper.inputDeviceReader.get_assisted_control() ==
                JoystickReader.ASSISTED_CONTROL_HEIGHTHOLD) or
                (self.helper.inputDeviceReader.get_assisted_control() ==
                 JoystickReader.ASSISTED_CONTROL_HOVER)):
            self.targetThrust.setEnabled(not enabled)
            self.targetHeight.setEnabled(enabled)
        else:
            self.helper.cf.param.set_value("flightmode.althold", str(enabled))

    @pyqtSlot(bool)
    def changeXmode(self, checked):
        self.helper.cf.commander.set_client_xmode(checked)
        Config().set("client_side_xmode", checked)
        logger.info("Clientside X-mode enabled: %s", checked)

    def alt1_updated(self, state):
        if state:
            new_index = (self._ring_effect+1) % (self._ledring_nbr_effects+1)
            self.helper.cf.param.set_value("ring.effect",
                                           str(new_index))

    def alt2_updated(self, state):
        self.helper.cf.param.set_value("ring.headlightEnable", str(state))

    def _ring_populate_dropdown(self):
        try:
            nbr = int(self.helper.cf.param.values["ring"]["neffect"])
            current = int(self.helper.cf.param.values["ring"]["effect"])
        except KeyError:
            return

        # Used only in alt1_updated function
        self._ring_effect = current
        self._ledring_nbr_effects = nbr

        hardcoded_names = {0: "Off",
                           1: "White spinner",
                           2: "Color spinner",
                           3: "Tilt effect",
                           4: "Brightness effect",
                           5: "Color spinner 2",
                           6: "Double spinner",
                           7: "Solid color effect",
                           8: "Factory test",
                           9: "Battery status",
                           10: "Boat lights",
                           11: "Alert",
                           12: "Gravity",
                           13: "LED tab"}

        for i in range(nbr + 1):
            name = "{}: ".format(i)
            if i in hardcoded_names:
                name += hardcoded_names[i]
            else:
                name += "N/A"
            self._led_ring_effect.addItem(name, i)

        self._led_ring_effect.currentIndexChanged.connect(
            self._ring_effect_changed)

        self._led_ring_effect.setCurrentIndex(current)
        if self.helper.cf.mem.ow_search(vid=0xBC, pid=0x01):
            self._led_ring_effect.setEnabled(True)
            self._led_ring_headlight.setEnabled(True)

    def _ring_effect_changed(self, index):
        self._ring_effect = index
        if index > -1:
            i = self._led_ring_effect.itemData(index)
            logger.info("Changed effect to {}".format(i))
            if i != int(self.helper.cf.param.values["ring"]["effect"]):
                self.helper.cf.param.set_value("ring.effect", str(i))

    def _ring_effect_updated(self, name, value):
        if self.helper.cf.param.is_updated:
            self._led_ring_effect.setCurrentIndex(int(value))

    def _populate_assisted_mode_dropdown(self):
        self._assist_mode_combo.addItem("Altitude hold", 0)
        self._assist_mode_combo.addItem("Position hold", 1)
        self._assist_mode_combo.addItem("Height hold", 2)
        self._assist_mode_combo.addItem("Hover", 3)
        heightHoldPossible = False
        hoverPossible = False

        if self.helper.cf.mem.ow_search(vid=0xBC, pid=0x09):
            heightHoldPossible = True

        if self.helper.cf.mem.ow_search(vid=0xBC, pid=0x0A):
            heightHoldPossible = True
            hoverPossible = True

        if not heightHoldPossible:
            self._assist_mode_combo.model().item(2).setEnabled(False)
        else:
            self._assist_mode_combo.model().item(0).setEnabled(False)

        if not hoverPossible:
            self._assist_mode_combo.model().item(3).setEnabled(False)
        else:
            self._assist_mode_combo.model().item(0).setEnabled(False)

        self._assist_mode_combo.currentIndexChanged.connect(
            self._assist_mode_changed)
        self._assist_mode_combo.setEnabled(True)

        try:
            assistmodeComboIndex = Config().get("assistedControl")
            if assistmodeComboIndex == 3 and not hoverPossible:
                self._assist_mode_combo.setCurrentIndex(0)
                self._assist_mode_combo.currentIndexChanged.emit(0)
            elif assistmodeComboIndex == 0 and hoverPossible:
                self._assist_mode_combo.setCurrentIndex(3)
                self._assist_mode_combo.currentIndexChanged.emit(3)
            elif assistmodeComboIndex == 2 and not heightHoldPossible:
                self._assist_mode_combo.setCurrentIndex(0)
                self._assist_mode_combo.currentIndexChanged.emit(0)
            elif assistmodeComboIndex == 0 and heightHoldPossible:
                self._assist_mode_combo.setCurrentIndex(2)
                self._assist_mode_combo.currentIndexChanged.emit(2)
            else:
                self._assist_mode_combo.setCurrentIndex(assistmodeComboIndex)
                self._assist_mode_combo.currentIndexChanged.emit(
                                                    assistmodeComboIndex)
        except KeyError:
            defaultOption = 0
            if hoverPossible:
                defaultOption = 3
            elif heightHoldPossible:
                defaultOption = 2
            self._assist_mode_combo.setCurrentIndex(defaultOption)
            self._assist_mode_combo.currentIndexChanged.emit(defaultOption)

    def _connected(self, link_uri):
        """Callback when the Crazyflie has been connected"""
        self._plot.removeAllDatasets()
        self._plot.set_title("")

    def _disconnected(self, link_uri):
        """Callback for when the Crazyflie has been disconnected"""
        self._model.beginResetModel()
        self._model.reset()
        self._model.endResetModel()
        self.dataSelector.setCurrentIndex(-1)
        self._previous_config = None
        self._started_previous = False

    def _log_data_signal_wrapper(self, ts, data, logconf):
        """Wrapper for signal"""

        # For some reason the *.emit functions are not
        # the same over time (?!) so they cannot be registered and then
        # removed as callbacks.
        self._log_data_signal.emit(ts, data, logconf)

    def _log_error_signal_wrapper(self, config, msg):
        """Wrapper for signal"""

        # For some reason the *.emit functions are not
        # the same over time (?!) so they cannot be registered and then
        # removed as callbacks.
        self._plotter_log_error_signal.emit(config, msg)

    def _selection_changed(self, i):
        """Callback from ComboBox when a new item has been selected"""

        # Check if we have disconnected
        if i < 0:
            return
        # First check if we need to stop the old block
        if self._started_previous and self._previous_config:
            logger.debug("Should stop config [%s], stopping!",
                         self._previous_config.name)
            self._previous_config.delete()

        # Remove our callback for the previous config
        if self._previous_config:
            self._previous_config.data_received_cb.remove_callback(
                self._log_data_signal_wrapper)
            self._previous_config.error_cb.remove_callback(
                self._log_error_signal_wrapper)

        lg = self._model.get_config(i)
        if not lg.started:
            logger.debug("Config [%s] not started, starting!", lg.name)
            self._started_previous = True
            lg.start()
        else:
            self._started_previous = False
        self._plot.removeAllDatasets()
        color_selector = 0

        self._plot.set_title(lg.name)

        for d in lg.variables:
            self._plot.add_curve(d.name, self.colors[
                color_selector % len(self.colors)])
            color_selector += 1
        lg.data_received_cb.add_callback(self._log_data_signal_wrapper)
        lg.error_cb.add_callback(self._log_error_signal_wrapper)

        self._previous_config = lg

    def _config_added(self, logconfig):
        """Callback from the log layer when a new config has been added"""
        logger.debug("Callback for new config [%s]", logconfig.name)
        self._model.add_block(logconfig)

    def _plotter_logging_error(self, log_conf, msg):
        """Callback from the log layer when an error occurs"""
        QMessageBox.about(
            self, "Plot error", "Error when starting log config [%s]: %s" % (
                log_conf.name, msg))

    def _log_data_received(self, timestamp, data, logconf):
        """Callback when the log layer receives new data"""

        # Check so that the incoming data belongs to what we are currently
        # logging
        if self._previous_config:
            if self._previous_config.name == logconf.name:
                self._plot.add_data(data, timestamp)
