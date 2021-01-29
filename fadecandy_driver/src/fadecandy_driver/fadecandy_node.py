import diagnostic_updater
import rospy
from fadecandy_msgs.msg import LEDArray

from .fadecandy_driver import FadecandyDriver


class FadecandyNode:
    def __init__(self):
        self._driver = None
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.add("Info", self._info_diagnostics)

        self._connect()
        self._set_leds_sub = rospy.Subscriber('set_leds', LEDArray, self._set_leds)
        self._diagnostic_timer = rospy.Timer(rospy.Duration(1), lambda e: self._diagnostic_updater.force_update())

    def _connect(self):
        if self._driver is not None:
            self._driver.release()
            self._driver = None

        rospy.loginfo('Connecting to Fadecandy device ..')

        first_try = True
        connection_retry_rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            try:
                self._driver = FadecandyDriver()
            except IOError as e:
                if first_try:
                    rospy.logwarn('Failed to connect to Fadecandy device: %s; will retry every second', e)
                    first_try = False
            else:
                rospy.loginfo('Connected to Fadecandy device')
                self._diagnostic_updater.setHardwareID(self._driver.serial_number)
                break
            connection_retry_rate.sleep()

    def _info_diagnostics(self, stat):
        if self._driver is None:
            stat.summary(diagnostic_updater.DiagnosticStatus.ERROR, 'Disconnected')
        else:
            stat.summary(diagnostic_updater.DiagnosticStatus.OK, 'Connected')
            stat.add('Serial number', self._driver.serial_number)

    def _set_leds(self, led_array_msg):
        if self._driver is None:
            return

        led_array_colors = []
        for led_strip_msg in led_array_msg.strips:
            led_strip_colors = [(int(c.r * 255), int(c.g * 255), int(c.b * 255)) for c in led_strip_msg.colors]
            led_array_colors.append(led_strip_colors)

        # Convert to a list of r, g, b tuples and pass to the driver.
        try:
            self._driver.set_colors(led_array_colors)
        except IOError as e:
            rospy.logerr('Failed to set colors: %s; reconnecting to device ..', e)
            self._connect()
