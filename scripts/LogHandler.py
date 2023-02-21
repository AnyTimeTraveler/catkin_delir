import logging
import LOGLEVEL

class LogHandler:
    def __init__(self, isRosUsed: bool):
        self.isRosUsed = isRosUsed
        if not self.isRosUsed:
            logging.basicConfig(level=logging.INFO)

    def log(self, msg: str, level: LOGLEVEL = LOGLEVEL.INFO):
        if self.isRosUsed:
            import rospy
            if level == LOGLEVEL.INFO:
                rospy.loginfo(msg)
            elif level == LOGLEVEL.DEBUG:
                rospy.logdebug(msg)
            elif level == LOGLEVEL.WARN:
                rospy.logwarn(msg)
            elif level == LOGLEVEL.ERROR:
                rospy.logerr(msg)
        else:
            if level == LOGLEVEL.INFO:
                logging.info(msg)
            elif level == LOGLEVEL.DEBUG:
                logging.debug(msg)
            elif level == LOGLEVEL.WARN:
                logging.warning(msg)
            elif level == LOGLEVEL.ERROR:
                logging.error(msg)

