import rospy

from fadecandy_ros.msg import LEDArray, LEDStrip
from fadecandy_ros.fadecandy_driver import FadecandyDriver

class FadecandyNode:
    def __init__(self):
        connection_retry_rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            try:
                self._driver = FadecandyDriver()
            except IOError:
                rospy.logwarn('Failed to connect to Fadecandy device; will retry every second')
            else:
                break
            connection_retry_rate.sleep()

        self._set_leds_sub = rospy.Subscriber('set_leds', LEDArray, self._setLEDs)

    def _setLEDs(self, led_array_msg):
        led_array_colors = []
        for led_strip_msg in led_array_msg.strips:
            led_strip_colors = [(c.r, c.g, c.b) for c in led_strip_msg.colors]
            led_array_colors.append(led_strip_colors)

        # Convert to a list of r, g, b tuples and pass to the driver.
        self._driver.setColors(led_array_colors)

