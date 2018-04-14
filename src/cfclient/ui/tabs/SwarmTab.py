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

import logging, time

from PyQt5 import uic
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal, QAbstractItemModel, QModelIndex
from PyQt5.QtWidgets import QMessageBox, QLabel
from PyQt5.QtGui import QPixmap

import cfclient
from cfclient.ui.widgets.ai import AttitudeIndicator
from cfclient.ui.widgets.plotwidget import PlotWidget

from cfclient.utils.config import Config
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cfclient.utils.periodictimer import PeriodicTimer
from cflib.utils.callbacks import Caller
from cflib.crazyflie.syncLogger import SyncLogger

from cfclient.utils.input import JoystickReader

from cfclient.ui.tab import Tab

LOG_NAME_ESTIMATED_Z = "stateEstimate.z"

__author__ = 'Bitcraze AB'
__all__ = ['SwarmTab']

logger = logging.getLogger(__name__)

example_tab_class = uic.loadUiType(cfclient.module_path +
                                  "/ui/tabs/swarmTab.ui")[0]

MAX_THRUST = 65536.0
INPUT_READ_PERIOD = 0.1

STYLE_RED_BACKGROUND = "background-color: lightpink;"
STYLE_GREEN_BACKGROUND = "background-color: lightgreen;"
STYLE_NO_BACKGROUND = "background-color: none;"


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
    disconnected_signal = pyqtSignal(str)
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

    # Change uris and sequences according to your setup
    URI1 = 'radio://0/80/2M/E7E7E7E701'
    URI2 = 'radio://0/80/2M/E7E7E7E702'
    URI3 = 'radio://0/80/2M/E7E7E7E703'
    URI4 = 'radio://0/80/2M/E7E7E7E704'
    URI5 = 'radio://0/80/2M/E7E7E7E705'
    URI6 = 'radio://0/80/2M/E7E7E7E706'
    URI7 = 'radio://0/80/2M/E7E7E7E707'
    URI8 = 'radio://0/80/2M/E7E7E7E708'
    URI9 = 'radio://0/80/2M/E7E7E7E709'
    URI10 = 'radio://0/80/2M/E7E7E7E70A'
    URIS = [
        URI1,
        URI2,
        URI3,
        URI4,
        URI5,
        URI6,
        URI7,
        URI8,
        URI9,
        URI10
    ]

    # List of URIs, comment the one you do not want to fly
    uris = {
        #URI1,
        URI2,
        #URI3,
        #URI4,
        URI5,
        URI6,
        #URI7,
        #URI8,
        #URI9,
        #URI10
    }

    # UI_DATA_UPDATE_FPS = 10

    connectionFinishedSignal = pyqtSignal(str)
    disconnectedSignal = pyqtSignal(str)

    _limiting_updated = pyqtSignal(bool, bool, bool)

    error_accumulator = [0] * len(URIS)
    _race_logger_signal = pyqtSignal(int, object , object)
    synclogger = None
    scoreboard_remap = None

    def __init__(self, tabWidget, helper, *args):
        super(SwarmTab, self).__init__(*args)
        self.setupUi(self)

        self.tabName = "Swarm"
        self.menuName = "Swarm"

        self.tabWidget = tabWidget
        self.helper = helper
        self.factory = CachedCfFactory(rw_cache='./cache')
        self.swarm = None
        self.cfs = [self.cf1,
                    self.cf2,
                    self.cf3,
                    self.cf4,
                    self.cf5,
                    self.cf6,
                    self.cf7,
                    self.cf8,
                    self.cf9,
                    self.cf10]
        self.status_cfs = [self.status_cf1,
                           self.status_cf2,
                           self.status_cf3,
                           self.status_cf4,
                           self.status_cf5,
                           self.status_cf6,
                           self.status_cf7,
                           self.status_cf8,
                           self.status_cf9,
                           self.status_cf10]

        for i in range(len(self.URIS)):
            self.cfs[i].setChecked(self.URIS[i] in self.uris)

        self.cf1.toggled.connect(lambda: self._select(0, self.cf1.isChecked()))
        self.cf2.toggled.connect(lambda: self._select(1, self.cf2.isChecked()))
        self.cf3.toggled.connect(lambda: self._select(2, self.cf3.isChecked()))
        self.cf4.toggled.connect(lambda: self._select(3, self.cf4.isChecked()))
        self.cf5.toggled.connect(lambda: self._select(4, self.cf5.isChecked()))
        self.cf6.toggled.connect(lambda: self._select(5, self.cf6.isChecked()))
        self.cf7.toggled.connect(lambda: self._select(6, self.cf7.isChecked()))
        self.cf8.toggled.connect(lambda: self._select(7, self.cf8.isChecked()))
        self.cf9.toggled.connect(lambda: self._select(8, self.cf9.isChecked()))
        self.cf10.toggled.connect(lambda: self._select(9, self.cf10.isChecked()))
        for i in range(10):
            label = getattr(self, 'status_cf{}'.format(i+1))
            label.setStyleSheet(STYLE_RED_BACKGROUND)
        self.swarmConnectBtn.clicked.connect(self.connected)
        self.raceTakeOffBtn.clicked.connect(self.race_take_off)
        self.raceStartBtn.clicked.connect(self.race_start)
        self.raceRaceBtn.clicked.connect(self.race_race)
        self.raceLandBtn.clicked.connect(self.race_land)
        self.disconnectedSignal.connect(self._disconnected)
        # self.helper.cfs.disconnected.add_callback(self.disconnectedSignal.emit)
        self.connectionFinishedSignal.connect(self._connected)
        # self.helper.cfs.cf.connected.add_callback(self.connectionFinishedSignal.emit)
        # self._race_logger_signal.connect(self.random_debugger)
        self.race_started = False
        self._read_timer = None
        self.RACE_STATE = 'LANDED'
        self.targetVfront = 0.0
        self.targetZ = 0.2
        self.tmp_timer = None
        self.tmp_counter = 0.0
        self.race_time = 10.0

        # setup gains UI
        self.k1Combo_cf1.valueChanged.connect(self._k123_gain_changed)
        self.k2Combo_cf1.valueChanged.connect(self._k123_gain_changed)
        self.k3Combo_cf1.valueChanged.connect(self._k123_gain_changed)

        self.k1Combo_cf2.valueChanged.connect(self._k123_gain_changed)
        self.k2Combo_cf2.valueChanged.connect(self._k123_gain_changed)
        self.k3Combo_cf2.valueChanged.connect(self._k123_gain_changed)

        self.k1Combo_cf3.valueChanged.connect(self._k123_gain_changed)
        self.k2Combo_cf3.valueChanged.connect(self._k123_gain_changed)
        self.k3Combo_cf3.valueChanged.connect(self._k123_gain_changed)

        self.updateGainsBtn.clicked.connect(self._controller_gain_wrapper)

    def _controller_gain_wrapper(self):
        self.swarm.sequential(self._controller_gain_changed)

    def _controller_gain_changed(self, scf):
        self.updateGainsBtn.setStyleSheet("background-color:#f0f0f0; color:#000000;")

        index = self.URIS.index(scf.cf.link_uri)
        i = self.scoreboard_remap.index(index)

        k1ComboLabel = getattr(self, 'k1Combo_cf{}'.format(i + 1))
        k2ComboLabel = getattr(self, 'k2Combo_cf{}'.format(i + 1))
        k3ComboLabel = getattr(self, 'k3Combo_cf{}'.format(i + 1))

        scf.cf.param.set_value("posCtlPid.zKi", str(k3ComboLabel.value()))
        scf.cf.param.set_value("posCtlPid.zKp", str(k1ComboLabel.value()))
        scf.cf.param.set_value("velCtlPid.vzKp", str(k2ComboLabel.value()))
        scf.cf.param.set_value("posCtlPid.zVelMax", str(15.0))
        scf.cf.param.set_value("velCtlPid.vzKd", str(0.0))
        scf.cf.param.set_value("velCtlPid.vzKi", str(0.0))
        scf.cf.param.set_value("posCtlPid.zKd", str(0.0))

    def _k123_gain_changed(self):
        logger.debug("controller gains updated")
        self.updateGainsBtn.setStyleSheet("background-color:#0078d7; color:#ffffff;")


    def _attach_connected_status(self, scf):
        scf.cf.connected.add_callback(self.connectionFinishedSignal.emit)
        scf.cf.disconnected.add_callback(self.disconnectedSignal.emit)

    def _select(self, nbr, checked):

        if self.URIS[nbr] not in self.uris and checked:
            logger.info("[cf%d] adding %s", nbr+1, self.URIS[nbr])
            self.uris.add(self.URIS[nbr])
        if self.URIS[nbr] in self.uris and not checked:
            logger.info("[cf%d] removing %s", nbr+1, self.URIS[nbr])
            self.uris.remove(self.URIS[nbr])
        logger.info(self.uris)

    def read_input_wrapper(self):
        self.swarm.parallel(self.read_input)

    def connected(self):
        if self.swarmConnectBtn.text() == 'Connect':
            self.swarm = Swarm(self.uris, factory=self.factory)
            self.helper.cfs = self.swarm._cfs.items()
            for uri, scf in self.helper.cfs:
                self._attach_connected_status(scf)
            self.swarm.__enter__()
            self.swarmConnectBtn.setText("Disconnect")
            self.swarm.parallel(self._attach_connected_status)
            self._read_timer = PeriodicTimer(INPUT_READ_PERIOD, self.read_input_wrapper)
            self._read_timer.start()
            self.swarm.sequential(self.attach_scoreboard_remap)
            # self.swarm.parallel(self.wait_for_param_download)
            self.swarm.parallel(self.attach_logger)
            # self.swarm.parallel(self.take_off)
            # self.swarm.parallel(self.hover)
            # self.swarm.parallel(self.land)
        else:
            self.tmp_timer = None
            self.swarm.close_links()
            self.swarm = None
            self.swarmConnectBtn.setText("Connect")

        # self.swarm.parallel(self.ping_crazyflies)
        # with Swarm(self.uris, factory=self.factory) as self.swarm:
        #     logger.info("hello")
        #     self.swarm.parallel(self.wait_for_param_download)

    def attach_scoreboard_remap(self, scf):
        index = self.URIS.index(scf.cf.link_uri)
        if self.scoreboard_remap is None:
            self.scoreboard_remap = [index]
            # self.scoreboard_remap = sorted(self.scoreboard_remap)
        else:
            self.scoreboard_remap += [index]
            self.scoreboard_remap = sorted(self.scoreboard_remap)

    def _disconnected(self, link_uri):
        i = self.URIS.index(link_uri) + 1
        label = getattr(self, 'status_cf{}'.format(i))
        label.setStyleSheet(STYLE_RED_BACKGROUND)

    def _connected(self, link_uri):
        i = self.URIS.index(link_uri) + 1
        label = getattr(self, 'status_cf{}'.format(i))
        label.setStyleSheet(STYLE_GREEN_BACKGROUND)

    def race_take_off(self):
        # self.swarm.parallel(self.take_off)
        # TODO:
        self.race_started = False
        self.RACE_STATE = 'TAKE_OFF'
        self.targetZ = 0.2
        self.tmp_timer = PeriodicTimer(INPUT_READ_PERIOD, self.take_off)
        self.tmp_timer.start()

    def race_start(self):
        self.race_started = False
        # self.tmp_timer.stop()
        # self.tmp_timer = PeriodicTimer(0.1, self.hover)
        # self.tmp_timer.start()
        # self.swarm.parallel(self.hover)
        self.RACE_STATE = 'HOVER'

    def race_race(self):
        self.race_started = True
        # self.tmp_timer.stop()
        # self.tmp_timer = PeriodicTimer(0.1, self.hover)
        # self.tmp_timer.start()
        # self.swarm.parallel(self.hover)
        self.tmp_timer.stop()
        self.tmp_counter = 0.0
        self.RACE_STATE = 'RACE'
        self.tmp_timer = PeriodicTimer(INPUT_READ_PERIOD, self.race)
        self.tmp_timer.start()

    def race_land(self):
        self.race_started = False
        self.RACE_STATE = 'LANDING'
        self.tmp_timer = PeriodicTimer(INPUT_READ_PERIOD, self.land)
        self.tmp_timer.start()
        # self.swarm.parallel(self.land)

    def read_input(self, scf):
        # logger.info("state is %s and targetZ is %f", self.RACE_STATE, self.targetZ)
        if not (self.RACE_STATE == 'LANDED'):
            if self.RACE_STATE == 'TAKE_OFF':
                if self.tmp_counter <= 1.0:
                    scf.cf.commander.send_hover_setpoint(0, 0, 0, self.targetZ)
                    time.sleep(0.01)
                else:
                    self.tmp_timer.stop()
                    self.RACE_STATE = 'HOVER'
                    self.tmp_counter = 0.0
            if self.RACE_STATE == 'HOVER':
                self.race_started = False
                scf.cf.commander.send_hover_setpoint(0, 0, 0, self.targetZ)
                # time.sleep(0.01)
            if self.RACE_STATE == 'RACE':
                if self.tmp_counter <= (self.race_time * 4):
                    scf.cf.commander.send_hover_setpoint(self.targetVfront, 0, 0, self.targetZ)
                else:
                    self.tmp_timer.stop()
                    self.RACE_STATE = 'HOVER'
                    self.tmp_counter = 0.0
            if self.RACE_STATE == 'LANDING':
                if self.targetZ > 0.2:
                    scf.cf.commander.send_hover_setpoint(0, 0, 0, self.targetZ)
                    # time.sleep(0.01)
                else:
                    time.sleep(0.1)
                    self.tmp_timer.stop()
                    self.RACE_STATE = 'LANDED'
                    self.tmp_counter = 0.0

    def take_off(self):
        take_off_time = 1.0
        sleep_time = INPUT_READ_PERIOD/2.0
        time_steps = int(take_off_time/ sleep_time)
        z_step = (0.6 - 0.2) / time_steps
        self.tmp_counter += sleep_time
        self.targetZ += z_step
        if self.targetZ >= 0.6:
            self.targetZ = 0.6

    def hover(self, scf):
        hover_time = 5.0
        sleep_time = INPUT_READ_PERIOD
        steps = int(hover_time/ sleep_time)
        targetZ = 0.6
        self.targetZ = 0.6

    def race(self):
        # logger.info("counter %f" % self.tmp_counter)
        race_time = self.race_time
        sleep_time = INPUT_READ_PERIOD
        race_speed = 0.2
        self.tmp_counter += sleep_time
        if self.tmp_counter < race_time:
            self.targetVfront = race_speed
        elif self.tmp_counter >race_time and self.tmp_counter <(2*race_time):
            self.targetVfront = -race_speed
        elif self.tmp_counter >(2*race_time) and self.tmp_counter <(3*race_time):
            self.targetVfront = race_speed
        elif self.tmp_counter >(3*race_time) and self.tmp_counter <(4*race_time):
            self.targetVfront = -race_speed

    def land(self):
        land_time = 1.0
        sleep_time = INPUT_READ_PERIOD/2.0
        time_steps = int(land_time/ sleep_time)
        z_step = (0.6 - 0.2) / time_steps
        self.tmp_counter += sleep_time
        self.targetZ -= z_step

    def attach_logger(self, scf):
        logger.info('Waiting for Logs')

        log_config = LogConfig(name='RaceLog', period_in_ms=20)
        log_config.add_variable('range.zrange', 'uint16_t')
        log_config.add_variable('stateEstimate.z', 'float')
        log_config.add_variable('posCtl.targetZ', 'float')

        index = self.URIS.index(scf.cf.link_uri)
        i = self.scoreboard_remap.index(index)
        logger.info('Index of CF %s is %s', index, i+1)
        label = getattr(self, 'uri_cf{}'.format(i + 1))
        label.setText(scf.cf.link_uri)

        try:
            scf.cf.log.add_config(log_config)
            log_config.data_received_cb.add_callback(
                lambda ts, data, lg: self._race_logger_signal_wrapper(ts, data, lg, index)
            )
            # log_config.error_cb.add_callback(self._log_error_signal.emit)
            log_config.start()
        except KeyError as e:
            logger.warning(str(e))
        except AttributeError as e:
            logger.warning(str(e))

        # self.synclogger = SyncLogger(scf, log_config)
        # logger.info('Waiting for Logs')
        # self.synclogger.__enter__()
        # logger.info('Waiting for Logs')
        # logger.info('Emitted %s', index)
        # self._race_logger_signal.emit(index)

        # for log_entry in self.synclogger[index]:
        #     data = log_entry[1]
        #
        #     # var_zRange = data['range.zrange']
        #     var_zState = data['stateEstimate.z']
        #     var_zTarget = data['posCtl.targetZ']
        #
        #     self.error_accumulator[index] += abs(var_zState - var_zTarget)
        #
        #     self._race_logger_signal.emit(index, self.error_accumulator[index])

            # logger.info('Accumulated Error %s', self.error_accumulator[index])
            # break

    def _race_logger_signal_wrapper(self, ts, data, lg, index):
        self._race_logger_signal.emit(ts, data, lg)
        # for log_entry in self.synclogger:
        var_zState = data['stateEstimate.z']
        var_zTarget = data['posCtl.targetZ']
        if self.race_started:
            self.error_accumulator[index] += abs(var_zState - var_zTarget)

            # logger.info('Error of CF %s is %s', index, self.error_accumulator[index])
        i = self.scoreboard_remap.index(index)
        label = getattr(self, 'error_cf{}'.format(i + 1))
        label.setText("%0.2f" % self.error_accumulator[index])
        # scoreboard_index =
        # logger.info('Accumulated Error of SCF %s is %s', index + 1, self.error_accumulator[index])

    def random_debugger(self, index, timestamp, data, logconf):
        logger.info('random debugger')
        # for log_entry in self.synclogger:
        var_zState = data['stateEstimate.z']
        var_zTarget = data['posCtl.targetZ']
        self.error_accumulator[5] += abs(var_zState - var_zTarget)
        # logger.info('Accumulated Error %s', self.error_accumulator[5])

    def wait_for_param_download(self, scf):
        logger.info('downloading params')
        while not scf.cf.param.is_updated:
            logger.info('while downloading params')
            time.sleep(1.0)
        logger.info('Parameters downloaded for %s', scf.cf.link_uri)

    def reset_estimator(self, scf):
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')

        self.wait_for_position_estimator(cf)

    # def take_off(self, cf, position):
    #     take_off_time = 1.0
    #     sleep_time = 0.1
    #     steps = int(take_off_time / sleep_time)
    #     vz = position[2] / take_off_time
    #
    #     print(vz)
    #
    #     for i in range(steps):
    #         cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
    #         time.sleep(sleep_time)

    # def land(self, cf, position):
    #     landing_time = 1.0
    #     sleep_time = 0.1
    #     steps = int(landing_time / sleep_time)
    #     vz = -position[2] / landing_time
    #
    #     print(vz)
    #
    #     for i in range(steps):
    #         cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
    #         time.sleep(sleep_time)
    #
    #     cf.commander.send_setpoint(0, 0, 0, 0)
    #     # Make sure that the last packet leaves before the link is closed
    #     # since the message queue is not flushed before closing
    #     time.sleep(0.1)

    def run_sequence(self, scf, sequence):
        try:
            cf = scf.cf
            cf.param.set_value('flightmode.posSet', '1')

            take_off(cf, sequence[0])
            for position in sequence:
                print('Setting position {}'.format(position))
                end_time = time.time() + position[3]
                while time.time() < end_time:
                    cf.commander.send_setpoint(position[1], position[0], 0,
                                               int(position[2] * 1000))
                    time.sleep(0.1)
            land(cf, sequence[-1])
        except Exception as e:
            print(e)


