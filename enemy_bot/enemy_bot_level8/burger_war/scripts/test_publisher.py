#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# 敵ポジション取得プログラム確認用
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped 

def para_in():
    # 初期化宣言 : このソフトウェアは"para_in"という名前
    rospy.init_node('para_in', anonymous=False)

    # nodeの宣言 : publisherのインスタンスを作る
    # input_dataというtopicにAdder型のmessageを送るPublisherをつくった
    pub = rospy.Publisher('relative_pose', PoseStamped, queue_size=100)

    # 1秒間にpublishする数の設定
    r = rospy.Rate(5)

    para_x = 2.6
    para_y = 0.0
    para_z = 0.0

    q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi)

    # Adder型のmessageのインスタンスを作る
    msg = PoseStamped()

    # ctl +　Cで終了しない限りwhileループでpublishし続ける

    while not rospy.is_shutdown():

        msg.pose.position.x = para_x
        msg.pose.position.y = para_y
        msg.pose.position.z = para_z
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        # publishする関数
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        para_in()

    except rospy.ROSInterruptException: pass