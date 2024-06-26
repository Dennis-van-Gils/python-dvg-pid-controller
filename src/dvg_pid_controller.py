#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PID controller with integral-windup & derivative-kick prevention and bumpless
manual-to-auto-mode transfer

Original C++ code by::

 /******************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 ******************************************************************************/

More information:
    * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
    * http://playground.arduino.cc/Code/PIDLibrary

Ported to Python by Dennis van Gils

Modifications:
    * Code refactoring.
    * P_ON_M mode has been removed.
    * Made the proportional, integrative and derivative terms accessible.
    * Made the last error accessible.
    * Added optional argument `differential_input` to have the PID controller
      regulate a specific difference with respect to the main argument
      `current_input`. The specific difference that will be regulated is set by
      `setpoint`.
      Contributor: https://github.com/antonverburg.

The 'compute' method should be called repeatedly at a fixed rate. Contrary to
the C library, the method here doesn't check the timing and just computes a
new PID output whenever it gets called. The I and D parameters are time rate
dependent by definition. I and D parameters passed to this class are on a per
second basis. 'Compute' will internally adjust I and D based on the actual
obtained time step between individual 'computes'.
"""
__author__ = "Dennis van Gils"
__authoremail__ = "vangils.dennis@gmail.com"
__url__ = "https://github.com/Dennis-van-Gils/python-dvg-pid-controller"
__date__ = "26-06-2024"
__version__ = "2.2.0"

from typing import Union
from enum import IntEnum

import time
import numpy as np
from dvg_debug_functions import dprint


class Direction(IntEnum):
    """Controller direction enumeration."""

    DIRECT = 1
    """Increasing output (+) leads to increasing input (+)."""

    REVERSE = -1
    """Increasing output (+) leads to decreasing input (-)."""


class Mode(IntEnum):
    """Controller mode enumeration."""

    MANUAL = 0
    """PID control is disabled."""

    AUTOMATIC = 1
    """PID control is enabled."""


class Constants:
    """Legacy code for backwards compatibility. Use enums `Direction` and `Mode`
    instead."""

    # fmt: off
    MANUAL    = 0   # Now `Mode.MANUAL`
    AUTOMATIC = 1   # Now `Mode.AUTOMATIC`
    DIRECT    = 1   # Now `Direction.DIRECT`
    REVERSE   = -1  # Now `Direction.REVERSE`
    # fmt: on


class PID_Controller:
    """Manages a PID controller with integral-windup & derivative-kick
    prevention and bumpless manual-to-auto-mode transfer.

    The `compute()` method should be called repeatedly, preferably at a fixed
    rate, to compute a new PID output whenever it gets called. The newly
    calculated PID output is available in attribute `output`. The Ki and Kd
    parameters are time dependent by definition and `compute()` will internally
    adjust the integral and derivative terms based on the actually obtained time
    step between individual 'computes'.

    The PID controller starts in manual mode, i.e. not enabled. You must call
    `set_mode(mode=Mode.Automatic, ...)` to start automatic PID control.

    Args:
        Kp (``float``):
            Proportional gain. Must be >= 0 in dimensionless units.

        Ki (``float``):
            Integral gain. Must be >= 0 in units of [sec^-1].

        Kd (``float``):
            Derivative gain. Must be >= 0 in units of [sec].

        direction (``int`` | ``Direction``, optional):
            Specify whether increasing output (+) leads to increasing input (+),
            aka 'direct', or the reverse.

            Default: ``Direction.Direct``

        debug (``bool``, optional):
            Print debug information to the terminal?

            Default: False

    Attributes:
        Kp (``float``, property):
            Set proportional gain in dimensionless units.

        Ki (``float``):
            Set integral gain in units of [sec^-1].

        Kd (``float``):
            Set derivative gain in units of [sec].

        output_limit_min (``float``):
            Minimum value to which the output and integral term will be clamped
            when running in `Automatic` mode. Part of the integral windup
            prevention.

        output_limit_max (``float``):
            Maximum value to which the output and integral term will be clamped
            when running in `Automatic` mode. Part of the integral windup
            prevention.

        in_auto (``bool``):
            Is the PID controller in Automatic mode, i.e. enabled?

        _output_P (``float``):
            Computed proportional term since the last call to `update()`.

        _output_I (``float``):
            Computed integral term since the last call to `update()`, adjusted
            for the actually obtained time step in between update calls and
            subsequently clamped to the set output limits.

        _output_D (``float``):
            Computed derivative term since the last call to `update()`, adjusted
            for the actually obtained time step in between update calls.

        output (``float``):
            Computed PID output since the last call to `update()`. It is the
            summation of the ``_output_P``, ``_output_I`` and ``_output_D`` terms,
            subsequently clamped to the set output limits.
    """

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        direction: Union[int, Direction] = Direction.DIRECT,
        debug: bool = False,
    ):
        self._setpoint = np.nan

        self._Kp = np.nan
        self._Ki = np.nan
        self._Kd = np.nan

        self._output_P = np.nan
        self._output_I = np.nan
        self._output_D = np.nan

        self.debug = debug
        """Print debug information to the terminal?"""

        # Must be set by set_output_limits()
        self.output_limit_min = np.nan
        self.output_limit_max = np.nan

        self.in_auto = False

        self.output = np.nan
        """Computed PID output since the last call to `update()`. It is the
        summation of the ``_output_P``, ``_output_I`` and ``_output_D`` terms, subsequently
        clamped to the set output limits."""

        self.set_tunings(Kp, Ki, Kd, direction)
        self.set_output_limits(0, 100)

        self.last_time = time.perf_counter()
        self.last_input = np.nan
        self.last_error = np.nan

    # --------------------------------------------------------------------------
    #   Properties
    # --------------------------------------------------------------------------

    def _throw_error_on_negative_value(self, value: float):
        if value < 0:
            raise ValueError(
                "Negative values not allowed. Set `direction = -1` instead."
            )

    @property
    def setpoint(self):
        """The setpoint to aim for."""
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value: float):
        self._setpoint = value

    @property
    def Kp(self):
        """The proportional gain in dimensionless units. Must be >= 0."""
        return self._Kp

    @Kp.setter
    def Kp(self, value: float):
        self._throw_error_on_negative_value(value)
        self._Kp = value

    @property
    def Ki(self):
        """The integral gain in units of [sec^-1]. Must be >= 0."""
        return self._Ki

    @Ki.setter
    def Ki(self, value: float):
        self._throw_error_on_negative_value(value)
        self._Ki = value

    @property
    def Kd(self):
        """The derivative gain in units of [sec]. Must be >= 0."""
        return self._Kd

    @Kd.setter
    def Kd(self, value: float):
        self._throw_error_on_negative_value(value)
        self._Kd = value

    @property
    def output_P(self):
        """Computed proportional term since the last call to `update()`."""
        return self._output_P

    @property
    def output_I(self):
        """Computed integral term since the last call to `update()`, adjusted
        for the actually obtained time step in between update calls and
        subsequently clamped to the set output limits.
        """
        return self._output_I

    @property
    def output_D(self):
        """Computed derivative term since the last call to `update()`, adjusted
        for the actually obtained time step in between update calls.
        """
        return self._output_D

    # --------------------------------------------------------------------------
    #   Methods
    # --------------------------------------------------------------------------

    def compute(
        self,
        current_input: float,
        differential_input: float = np.nan,
    ) -> bool:
        """Compute new PID output. This function should be called repeatedly,
        preferably at a fixed time interval.

        Optional argument `differential_input` can be used to have the PID
        controller regulate a specific difference with respect to the main
        argument `current_input`. The specific difference that will be regulated
        is set by `setpoint`.
        Use-case example: Regulating the difference between input- and output
        water temperatures to be about 2 degC.

        Returns True when the output is computed, false when nothing has been
        done.
        """

        now = time.perf_counter()  # [s]
        time_step = now - self.last_time

        if (not self.in_auto) or np.isnan(self._setpoint):
            self.last_time = now
            return False

        _input = current_input

        if not np.isnan(differential_input):
            _input = differential_input - current_input
        self.last_error = self._setpoint - _input

        # Proportional term
        self._output_P = self.direction * self._Kp * self.last_error

        # Integral term
        self._output_I = self._output_I + (
            self.direction * self._Ki * time_step * self.last_error
        )

        if self.debug:
            if self._output_I < self.output_limit_min:
                dprint("_output_I < output_limit_min: integral windup")
            elif self._output_I > self.output_limit_max:
                dprint("_output_I > output_limit_max: integral windup")

        # Prevent integral windup
        self._output_I = np.clip(
            self._output_I, self.output_limit_min, self.output_limit_max
        )

        # Derivative term
        # Prevent derivative kick: really good to do!
        self._output_D = (
            -(self.direction * self._Kd)
            / time_step
            * (_input - self.last_input)
        )

        # Compute PID Output
        self.output = self._output_P + self._output_I + self._output_D

        if self.debug:
            dprint(f"{time_step * 1000}")
            dprint(
                f"{self._output_P:.1f} {self._output_I:.1f} {self._output_D:.1f}"
            )
            dprint((" " * 14 + f"{self.output:.2f}"))

            if self.output < self.output_limit_min:
                dprint("output < output_limit_min: output clamped")
            elif self.output > self.output_limit_max:
                dprint("output > output_limit_max: output clamped")

        # Clamp the output to its limits
        self.output = np.clip(
            self.output, self.output_limit_min, self.output_limit_max
        )

        # Remember some variables for next time
        self.last_input = _input
        self.last_time = now

        return True

    def set_tunings(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        direction: Union[int, Direction] = Direction.DIRECT,
    ):
        """This function allows the controller's dynamic performance to be
        adjusted. It's called automatically from the constructor, but tunings
        can also be adjusted on the fly during normal operation.

        The PID will either be connected to a DIRECT acting process
        (+Output leads to +Input) or a REVERSE acting process (+Output leads to
        -Input). We need to know which one, because otherwise we may increase
        the output when we should be decreasing. This is called from the
        constructor.
        """

        if (Kp < 0) or (Ki < 0) or (Kd < 0):
            return

        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self.direction = direction

    def set_output_limits(
        self,
        limit_min: float,
        limit_max: float,
    ):
        if limit_min >= limit_max:
            return

        self.output_limit_min = limit_min
        self.output_limit_max = limit_max

        if self.in_auto:
            self.output = np.clip(self.output, limit_min, limit_max)
            self._output_I = np.clip(self._output_I, limit_min, limit_max)

    def set_mode(
        self,
        mode: Union[int, Mode],
        current_input: float,
        current_output: float,
        differential_input: float = np.nan,
    ):
        """Allows the controller Mode to be set to manual (0) or Automatic
        (non-zero). When the transition from manual to auto occurs, the
        controller is automatically initialized.
        """

        new_auto = mode == Mode.AUTOMATIC
        if new_auto and not self.in_auto:
            # We just went from manual to auto
            self.initialize(current_input, current_output, differential_input)

        self.in_auto = new_auto

    def initialize(
        self,
        current_input: float,
        current_output: float,
        differential_input=np.nan,
    ):
        """Does all the things that need to happen to ensure a bumpless
        transfer from manual to automatic mode.
        """
        self._output_I = current_output
        if np.isnan(differential_input):
            self.last_input = current_input
        else:
            self.last_input = differential_input - current_input

        if self.debug:
            dprint("PID init")
            if self._output_I < self.output_limit_min:
                dprint(
                    "@PID init: _output_I < output_limit_min: integral windup"
                )
            elif self._output_I > self.output_limit_max:
                dprint(
                    "@PID init: _output_I > output_limit_max: integral windup"
                )

        self._output_I = np.clip(
            self._output_I, self.output_limit_min, self.output_limit_max
        )
