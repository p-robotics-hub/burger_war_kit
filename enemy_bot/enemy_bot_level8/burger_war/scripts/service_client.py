import rospy
from std_srvs.srv import Empty
import time

if __name__ == "__main__":
    rospy.wait_for_service("service_call")
    service_call = rospy.ServiceProxy("service_call",Empty)
    while True:
        # 1秒に１回サービスで通信
        service_call()
        time.sleep(1)
        