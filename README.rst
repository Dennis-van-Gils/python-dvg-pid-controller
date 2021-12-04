.. image:: https://img.shields.io/pypi/v/dvg-pid-controller
    :target: https://pypi.org/project/dvg-pid-controller
.. image:: https://img.shields.io/pypi/pyversions/dvg-pid-controller
    :target: https://pypi.org/project/dvg-pid-controller
.. image:: https://requires.io/github/Dennis-van-Gils/python-dvg-pid-controller/requirements.svg?branch=master
     :target: https://requires.io/github/Dennis-van-Gils/python-dvg-pid-controller/requirements/?branch=master
     :alt: Requirements Status
.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
.. image:: https://img.shields.io/badge/License-MIT-purple.svg
    :target: https://github.com/Dennis-van-Gils/python-dvg-pid-controller/blob/master/LICENSE.txt

DvG_PID_Controller
==================
*PID controller with integral-windup & derivative-kick prevention and bumpless
manual-to-auto-mode transfer.*

Installation::

    pip install dvg-pid-controller

Based on C++ code by:

    ::

        /******************************************************************************
        * Arduino PID Library - Version 1.2.1
        * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
        *
        * This Library is licensed under the MIT License
        ******************************************************************************/

    More information:
        * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
        * http://playground.arduino.cc/Code/PIDLibrary

    Ported to Python by Dennis van Gils.

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
