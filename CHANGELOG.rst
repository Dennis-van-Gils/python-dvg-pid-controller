Changelog
=========

1.0.0 (2020-06-29)
------------------
* First release on PyPI

1.0.1 (2020-06-29)
------------------
* Fixed Readme typo

2.0.0 (2020-07-02)
------------------
* DvG module filenames changed to lowercase

2.0.1 (2021-12-03)
------------------
(yanked, should have been minor version increment instead of patch)

2.0.2 (2021-12-04)
------------------
(yanked, should have been minor version increment instead of patch)

2.1.0 (2021-12-04)
------------------
* Made the last error accessible
* Added optional argument `differential_input` to have the PID controller
  regulate a specific difference with respect to the main argument
  `current_input`. The specific difference that will be regulated is set by
  `setpoint`.
  Contributor: https://github.com/antonverburg.