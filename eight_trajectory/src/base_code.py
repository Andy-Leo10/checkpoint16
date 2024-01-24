#! /usr/bin/env python

import rospy, numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def odom_callback(msg):
    global phi_,x_,y_
    position = msg.pose.pose.position
    x_ = position.x
    y_ = position.y
    (_, _, phi_) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                         msg.pose.pose.orientation.y, 
                                         msg.pose.pose.orientation.z, 
                                         msg.pose.pose.orientation.w])
    #rospy.loginfo("x: %f, y: %f, phi: %f", x_, y_, phi_)
    
rospy.init_node('absolute_motion', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
position_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.sleep(1)

def velocity2twist(dphi, dx, dy,scale_velocities=False):
    R = np.array([[1, 0, 0],
                  [0,  np.cos(phi_), np.sin(phi_)],
                  [0, -np.sin(phi_), np.cos(phi_)]])
    v = np.array([dphi, dx, dy])
    v.shape = (3,1)
    twist = np.dot(R, v)
    wz, vx, vy = twist.flatten().tolist()
    #rospy.loginfo("wz: %.2f, vx: %.2f, vy: %.2f", wz, vx, vy)
    if scale_velocities:
        v_const = 0.5
        w_const = 0.5
        sv = v_const / np.max(np.abs([vx, vy])) if np.max(np.abs([vx, vy])) > 0 else 1
        sw = w_const / np.abs(wz) if np.abs(wz) > 0 else 1
        vx *= sv
        vy *= sv
        wz *= sw
    return wz, vx, vy

def twist2wheels(wz, vx, vy):
    # half of the wheel base distance
    l = 0.500/2
    # the radius of the wheels
    r = 0.254/2
    # half of track width
    w = 0.548/2
    
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r
    twist = np.array([wz, vx, vy])
    twist.shape = (3,1)
    u = np.dot(H, twist)
    u = u.flatten().tolist()
    #rospy.loginfo("u1: {:.2f}, u2: {:.2f}, u3: {:.2f}, u4: {:.2f}, ".format(u[0],u[1],u[2],u[3]))
    return u

def move_to_target(pub, desired_phi=30, desired_x=0.5, desired_y=1.0):
    global phi_, x_, y_  # Declare the variables as global
    
    # Calculate the difference in odometry
    dphi = np.deg2rad(desired_phi)-phi_
    dx = desired_x-x_
    dy = desired_y-y_
    rospy.loginfo("dphi: %.2f, dx: %.2f, dy: %.2f", dphi, dx, dy)

    # Define a threshold for the position difference
    threshold = 0.01

    while abs(dx) > threshold or abs(dy) > threshold or abs(dphi) > threshold:
        wz, vx, vy = velocity2twist(dphi, dx, dy)
        u = twist2wheels(wz, vx, vy)
        msg = Float32MultiArray(data=u)
        pub.publish(msg)
        rospy.sleep(0.01)

        # Update the difference in odometry
        dphi = np.deg2rad(desired_phi)-phi_
        dx = desired_x-x_
        dy = desired_y-y_
        
    stop = [0,0,0,0]
    msg = Float32MultiArray(data=stop)
    pub.publish(msg)

move_to_target(pub, 60, 1.0, 0.5)
move_to_target(pub, 150, 1.0, -0.5)
move_to_target(pub, -90, -1.0, 1.5)

print("Done")
# # Calculate the difference in odometry
# dphi = np.deg2rad(30)-phi_
# dx = 0.5-x_
# dy = 1.0-y_
# rospy.loginfo("dphi: %.2f, dx: %.2f, dy: %.2f", dphi, dx, dy)

# # Define a threshold for the position difference
# threshold = 0.01

# while abs(dx) > threshold or abs(dy) > threshold or abs(dphi) > threshold:
#     wz, vx, vy = velocity2twist(dphi, dx, dy)
#     u = twist2wheels(wz, vx, vy)
#     msg = Float32MultiArray(data=u)
#     pub.publish(msg)
#     rospy.sleep(0.01)

#     # Update the difference in odometry
#     dphi = np.deg2rad(30)-phi_
#     dx = 0.5-x_
#     dy = 1.0-y_
    
# stop = [0,0,0,0]
# msg = Float32MultiArray(data=stop)
# pub.publish(msg)
