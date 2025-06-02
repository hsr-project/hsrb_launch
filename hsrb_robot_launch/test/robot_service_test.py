#!/usr/bin/env python
# Copyright (c) 2024 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# -*- coding: utf-8 -*-
u"""
Test Item:

 * Focus on testing RobotService.run()

Check the restart loop: test_run_hsrb, test_run_hsrc_emergency, test_run_hsrc_wireless
When starting for the first time:
- launch_nodes(True) is called
After watching:
- Restart 1: When it stopped with emergency stop being True
    term_nodes(False), launch_nodes(False) is called (partially stopped, restarted)
- Restart 2: When it stopped with emergency stop being True, and then there is no operation for the specified time (specified by keep_nodes_time)
    term_nodes(True), launch_nodes(True) is called (all nodes stopped, restarted)
- Restart 3: When it stops as no topic from emergency stop came for a specified seconds (specified by watch_timeout)
    term_nodes(True), launch_nodes(True) is called (all nodes stopped, restarted)

 Test for default speech (Japanese): test_notify_default_ja
 On start
 - Speech "HSR start"
 On stop
 - Speech "Stopped"

 Test for PERSONAL_NAME setting speech (Japanese): test_notify_original_ja
 Example: PERSONAL_NAME=test
 On start
 - Speech "Test start"
 On stop
 - Speech "Test, stopped"

 Test for default speech (English): test_notify_default_en
 On start
 - Speech "HSR start"
 On stop
 - Speech "HSR stopped"

 Test for PERSONAL_NAME setting speech on start (English): test_notify_original_en
 Example: PERSONAL_NAME=test
 On start
 - Speech "Test start"
 On stop
 - Speech "Test stopped"
"""

import os
from sys import float_info
import threading
from unittest.mock import MagicMock

from diagnostic_msgs.msg import DiagnosticArray
from hsrb_robot_launch.robot_service import HsrbRobotService, HsrcRobotService
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

from tmc_voice_msgs.msg import Voice


def wait_until(node, condition_function, timeout_sec=5.0, rate_hz=100.0):
    if not callable(condition_function):
        raise TypeError('Input condition_function must be callable')
    if timeout_sec <= 0.0:
        # https://docs.python.org/ja/3/library/exceptions.html
        # When an operator or a function receives an argument that is of the correct type but has an inappropriate value,
        # it is raised in situations where it cannot be described by a more specific exception like IndexError.
        raise ValueError('Timeout value must be positiv')
    if rate_hz < float_info.epsilon:
        raise ValueError('Evaluation frequency must be great enough')

    endtime = node.get_clock().now() + rclpy.time.Duration(seconds=timeout_sec)
    rate = node.create_rate(rate_hz)
    while rclpy.ok():
        if condition_function():
            return True
        if node.get_clock().now() > endtime:
            break
        rate.sleep()

    return False


@pytest.fixture
def setup_node(mocker):
    rclpy.init()
    test_node = rclpy.create_node('test_node')
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(test_node, executor), daemon=True)
    thread.start()

    mocker.patch('signal.signal')
    mocker.patch('atexit.register')
    mocker.patch('rclpy.init')

    yield test_node

    test_node.destroy_node()
    rclpy.shutdown()
    thread.join()


class TestRobotService:
    def exit(self):
        self.service.running = False
        self.run_thread.join()
        self.service.node.destroy_node()

    def publish(self, publisher, data):
        message = Bool()
        message.data = data
        publisher.publish(message)

    def exec_run_method(self, robot_name='hsrb', watch_timeout=15, keep_nodes_time=0.05, lang=None):
        if robot_name == 'hsrb':
            self.service = HsrbRobotService(watch_timeout=watch_timeout, keep_nodes_time=keep_nodes_time, lang=lang)
        else:
            self.service = HsrcRobotService(watch_timeout=watch_timeout, keep_nodes_time=keep_nodes_time, lang=lang)

        self.service.term_nodes = MagicMock()
        self.service.launch_nodes = MagicMock()
        self.service.diag_pub = MagicMock()
        self.service.wait_until_servo_is_not_ready = MagicMock()
        self.service.wait_until_servo_is_ready = MagicMock()
        if lang is None:
            self.service.notify = MagicMock()

        self.run_thread = threading.Thread(target=self.service.run, daemon=True)
        self.run_thread.start()

        # Wait for the node to start, check the arguments of the initial start
        wait_until(self._node, lambda: not self.service.launch_nodes.call_count == 0)
        self.service.launch_nodes.assert_called_with(True)

    def check_restart_nodes(self, count, require_called):
        u"""Wait for the node to fall and check the arguments, then wait for it to rise and check the arguments"""
        wait_until(self._node, lambda: not self.service.term_nodes.call_count == count)
        self.service.term_nodes.assert_called_with(require_called)

        wait_until(self._node, lambda: not self.service.launch_nodes.call_count == count + 1)

        self.service.launch_nodes.assert_called_with(require_called)

    def _test_run_normal_impl(self, setup_node, robot_name, publisher):
        self._node = setup_node
        callback = MagicMock()
        self._node.create_subscription(Voice, 'talk_request', callback, 1)
        self._node.create_subscription(DiagnosticArray, 'diagnostics', callback, 1)
        self.exec_run_method(robot_name=robot_name, watch_timeout=3)

        # Emergency stop button pressed
        self.publish(publisher, True)
        self._node.create_rate(5).sleep()
        # Confirm that some nodes are restarted when the servo turns ON
        self.publish(publisher, False)
        self.check_restart_nodes(0, True)

        # Emergency stop button pressed
        self.publish(publisher, True)
        self._node.create_rate(5).sleep()
        # If there is no operation for 3 seconds (60.0 x 0.05 (s)), all nodes will stop
        self.check_restart_nodes(1, True)

        # If more than 3 seconds elapse without a signal from the emergency stop, all nodes will stop (assuming the issuing node is dead)
        self.check_restart_nodes(2, True)

        self.exit()

    def test_run_hsrb(self, setup_node):
        runstop_pub = setup_node.create_publisher(Bool, "runstop_button", 1)
        self._test_run_normal_impl(setup_node, robot_name="hsrb", publisher=runstop_pub)

    def test_run_hsrc_emergency(self, setup_node):
        emergency_stop_pub = setup_node.create_publisher(Bool, "emergency_stop_button", 1)
        self._test_run_normal_impl(setup_node, robot_name="hsrc", publisher=emergency_stop_pub)

    def test_run_hsrc_wireless(self, setup_node):
        wireless_stop_pub = setup_node.create_publisher(Bool, "wireless_stop_button", 1)
        self._test_run_normal_impl(setup_node, robot_name="hsrc", publisher=wireless_stop_pub)

    def _check_voice(self, callback, call_count, message):
        wait_until(self._node, lambda: callback.call_count == call_count)
        assert callback.call_args[0][0].sentence == message

    def _test_notify_impl(self, setup_node, lang, name):
        voice_messages = {'ja': {'default': ['HSR' + u'Start', u'Stopped'],
                                 'test': ['Test' + u'Start', u'Test' + u',Stoppd']},
                          'en': {'default': [u'HSR start', u'HSR stopped'],
                                 'test': [u'test start', u'test stopped']}}

        self._node = setup_node
        runstop_pub = self._node.create_publisher(Bool, "runstop_button", 1)
        callback_talk = MagicMock()
        self._node.create_subscription(Voice, 'talk_request', callback_talk, 1)
        callback_diag = MagicMock()
        self._node.create_subscription(DiagnosticArray, 'diagnostics', callback_diag, 1)
        self.exec_run_method(keep_nodes_time=10, lang=lang)

        # Speech test at startup
        self._check_voice(callback_talk, 1, voice_messages[lang][name][0])

        # Speech test at shutdown
        self.publish(runstop_pub, True)
        self._check_voice(callback_talk, 2, voice_messages[lang][name][1])

        self.exit()

    def test_notify_default_ja(self, setup_node):
        u"""Test for default speech (Japanese)"""
        self._test_notify_impl(setup_node, 'ja', 'default')

    def test_notify_original_ja(self, setup_node):
        u"""Test for PERSONAL_NAME setting speech (Japanese)"""
        os.environ['PERSONAL_NAME'] = u'test'
        self._test_notify_impl(setup_node, 'ja', 'test')
        os.environ.pop('PERSONAL_NAME', None)

    def test_notify_default_en(self, setup_node):
        u"""Test for default speech (English)"""
        self._test_notify_impl(setup_node, 'en', 'default')

    def test_notify_original_en(self, setup_node):
        u"""Test for PERSONAL_NAME setting speech (English)"""
        os.environ['PERSONAL_NAME'] = 'test'
        self._test_notify_impl(setup_node, 'en', 'test')
        os.environ.pop('PERSONAL_NAME', None)
