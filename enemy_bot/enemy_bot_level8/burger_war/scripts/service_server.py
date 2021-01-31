import rospy
from std_srvs.srv import Empty,EmptyResponse

def callback_srv(req):
    print "Received"
    # FlagをTrueにするとか
    return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node("srv_server")
    srv = rospy.Service("service_call", Empty, callback_srv)
    rospy.spin()