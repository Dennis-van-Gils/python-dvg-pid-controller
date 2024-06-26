Changelog
=========

2.2.0 (2024-06-26)
------------------
* Support for Numpy 2.0 without any change needed
* That being done, this library is now abandoned. The implementation found at
  https://github.com/m-lundberg/simple-pid is more elegant.

2.1.1 (2023-02-27)
------------------
* Deprecated `requires.io`

2.1.0 (2021-12-04)
------------------
* Made the last error accessible
* Added optional argument `differential_input` to have the PID controller
  regulate a specific difference with respect to the main argument
  `current_input`. The specific difference that will be regulated is set by
  `setpoint`.
  Contributor: https://github.com/antonverburg.

2.0.2 (2021-12-04)
------------------
(yanked, should have been minor version increment instead of patch)

2.0.1 (2021-12-03)
------------------
(yanked, should have been minor version increment instead of patch)

2.0.0 (2020-07-02)
------------------
* DvG module filenames changed to lowercase

1.0.1 (2020-06-29)
------------------
* Fixed Readme typo

1.0.0 (2020-06-29)
------------------
* First release on PyPI