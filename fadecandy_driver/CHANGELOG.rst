^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fadecandy_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Feat/refactor to c++ (`#20 <https://github.com/iron-ox/fadecandy_ros/issues/20>`_)
  Co-authored-by: jad <jad.hajmustafa@eurogroep.com>
  Co-authored-by: Rein Appeldoorn <reinzor@gmail.com>
* Merge pull request `#17 <https://github.com/iron-ox/fadecandy_ros/issues/17>`_ from eurogroep/fix/handle-reconnections
  fix: Handle reconnections properly
* fix(diagnostics): Publish before first connect
* Merge pull request `#15 <https://github.com/iron-ox/fadecandy_ros/issues/15>`_ from eurogroep/fix/remove-fadecandy-util-unused-file
  fix: remove unused fadecandy_util.py
* fix: Handle reconnections properly
  Release the USB device and restart the driver when an IOError has been
  raised while sending to the USB device. This way, we can recover from
  unplugging and pluggin the USB device.
* fix: remove unused fadecandy_util.py
* Contributors: Catherine Wong, Jad Haj Mustafa, Rein Appeldoorn

0.1.3 (2020-11-02)
------------------
* Merge pull request `#13 <https://github.com/iron-ox/fadecandy_ros/issues/13>`_ from eurogroep/fix/noetic-python3-struct-pack: fix(Noetic/Python3): Struct packing
* Contributors: Rein Appeldoorn

0.1.2 (2020-07-06)
------------------
* Merge pull request `#10 <https://github.com/iron-ox/fadecandy_ros/issues/10>`_ from eurogroep/chore/log-io-error
  chore: log IO error
* Merge pull request `#9 <https://github.com/iron-ox/fadecandy_ros/issues/9>`_ from eurogroep/fix/rospy-shutdown
  fix(shutdown): Shutdown gracefully when no connection was set-up
* Contributors: Rein Appeldoorn

0.1.1 (2020-06-03)
------------------
* Merge pull request `#7 <https://github.com/iron-ox/fadecandy_ros/issues/7>`_ from jonbinney/python3-fixes
  Fixes for python3/noetic compatibility
* Contributors: Jon Binney

0.1.0 (2020-05-28)
------------------

- Initial release.
