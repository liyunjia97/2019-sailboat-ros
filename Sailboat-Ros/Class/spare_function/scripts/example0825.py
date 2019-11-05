#!/usr/bin/env python
# -*- coding: utf-8-*
import rospy

from sailboat_message.msg import Sensor_msg
from sailboat_message.msg import Mach_msg

from spare_function.cfg import spare_function_Config
from spare_function.msg import spare_function_out
from spare_function.msg import spare_function_para
from collections import deque
from collections import Counter

from dynamic_reconfigure.server import Server
import math
import numpy as np

# from numpy import sin,cos,pi,arctan2,sqrt,rad2deg
sensor_submsg = [0,0,0,0,0,0,0,0,0,0]
para_cfg = [0,0,0,0,0,0,0,0]

sa_out = deque(maxlen=10)
sa_out.append(0)#输出帆角        
heading_error = deque(maxlen=100)
heading_error.append(0)
twx_list=[0]
twy_list=[0]
tws=0
twa=0
LABEL=0
LABEL_TIME=0

def aw2tw(aws, awa, u, v, heading):
	vx = u*np.cos(heading)-v*np.sin(heading)
	vy = u*np.sin(heading)+v*np.cos(heading)
	twx = aws*math.cos(awa+heading+np.pi)+vx
	twy = aws*math.sin(awa+heading+np.pi)+vy
	return twx, twy

def goal_angle_func(ship_x, ship_y, heading, goal_x, goal_y,twa):
	goal_angle = math.atan2(goal_y-ship_y, goal_x-ship_x)
	goal_wind = angle_limit(goal_angle - twa)

	return goal_angle,goal_wind

def sa_goal_func(twa, goal_wind, heading):
	wha =angle_limit(twa-heading)
	if wha>np.pi/8 and wha<=np.pi*3/8:
		sa_goal = wha-5/8*np.pi
	elif wha>-np.pi*3/8 and wha<=-np.pi/8:
		sa_goal = wha+5/8*np.pi
	elif wha>np.pi*3/8 and wha<=np.pi*7/8:
		sa_goal = 0.5*wha-np.pi*7/16
	elif wha>-np.pi*7/8 and wha<=-np.pi*3/8:
		sa_goal = 0.5*wha+np.pi*7/16
	elif abs(wha)>np.pi*7/8 and abs(wha)<=np.pi:
		sa_goal = 0
	else:
		if goal_wind > 0:
			sa_goal = np.pi/2
		else: 
			sa_goal = -np.pi/2
	return sa_goal

def sa_func(heading_error, sa_goal):
	skp = para_cfg[5]
	ski = para_cfg[6]
	skd = para_cfg[7]

	sa_output = sa_goal+skp*heading_error[-1]+skd*(heading_error[-1]-heading_error[-2])+ski*(sum(heading_error))
	return sa_output # 返回控制帆角值

# heading_goal_func 正常 jibing1右直行 jibing2左直行
def heading_goal_func(goal_angle, heading,twa):
	heading_twa=angle_limit(heading-twa-np.pi)
	goal_twa=angle_limit(goal_angle-twa-np.pi)
	heading_goal=goal_twa-heading_twa
	return heading_goal

def ra_func(heading_error,u):
	ra_limit_0=0.15
	u_up=0.3
	u_down=0.1
	#根据航速限制舵角上限
	if u >u_up:
		ra_limit=ra_limit_0
	elif u>u_down and u<u_up:
		ra_limit=(u-u_down)/(u_up-u_down)*ra_limit_0
	else:
		ra_limit=0
	rkp = para_cfg[2]
	rki = para_cfg[3]
	rkd = para_cfg[4]

	ra_output = rkp*heading_error[-1]+rkd*(heading_error[-1]-heading_error[-2])+rki*(sum(heading_error))
	if ra_output>ra_limit:
		ra_output=ra_limit
	elif ra_output<-ra_limit:
		ra_output=-ra_limit
	return ra_output

# 角度限制（限制在(-pi, pi]范围内）
def angle_limit(angle):
	if angle > np.pi:
		angle -= 2*np.pi
	if angle <= -np.pi:
		angle += 2*np.pi
	return angle

# 根据传感器输入规划帆船的帆角sa和舵角ra
def module(sensor_submsg,goal):
	global LABEL, point10_x, point10_y, tws, twa, twx_list, twy_list, LABEL_TIME
	u, v, r, dheel, x, y, heading, heel,aws,awa=sensor_submsg[0], sensor_submsg[1], sensor_submsg[2], sensor_submsg[3], sensor_submsg[4], sensor_submsg[5], sensor_submsg[6], sensor_submsg[7],sensor_submsg[8],sensor_submsg[9]
	if heading<np.pi:
		heading=heading
	else:
		heading=heading-2*np.pi
	twx_temp, twy_temp = aw2tw(aws, awa, u, v, heading)
	twx_list.append(twx_temp)
	twy_list.append(twy_temp)
	if len(twx_list)>10:
		twx=np.mean(twx_list)  #前10个点平均计算风速
		twy=np.mean(twy_list)
		twx_list=[0]
		twy_list=[0]
		tws = np.sqrt(twx*twx+twy*twy)
		if awa>0:
			twa = angle_limit(math.atan2(twy, twx)+np.pi)
		else:
			twa = angle_limit(math.atan2(twy, twx)-np.pi)
	print('heading',heading,'twa',twa,'awa',awa)
	goal_angle,goal_wind = goal_angle_func(x,y,heading, goal[0],goal[1],twa)
	twa_limit=np.pi*1/3#禁航区范围
	goal_twa_limit=np.pi*3/4#逆风跑的弧度,twa坐标系

	if LABEL == 1: #右前直行
		if abs(goal_wind)>twa_limit:
			LABEL=11
			point10_x=x
			point10_y=y
			LABEL_TIME=1
		heading_twa=angle_limit(heading-twa-np.pi)
		goal_twa=-goal_twa_limit
		heading_goal=goal_twa-heading_twa
		if abs(heading_goal)<np.pi/6:
			sa_goal=sa_goal_func(twa, goal_wind, heading)
		else:
			sa_goal=sa_goal_func(twa, goal_wind, heading)

	elif LABEL == 11:#右前执行延长5s
		LABEL_TIME += 1
		if LABEL_TIME>50:
			LABEL=0
		if abs(goal_wind)>np.pi/2:
			LABEL=0
		heading_twa=angle_limit(heading-twa-np.pi)
		goal_twa=-goal_twa_limit
		heading_goal=goal_twa-heading_twa
		if abs(heading_goal)<np.pi/6:
			sa_goal=sa_goal_func(twa, goal_wind, heading)
		else:
			sa_goal=sa_goal_func(twa, goal_wind, heading)

	elif LABEL == 2:#左前直行
		if abs(goal_wind)>twa_limit:
			LABEL=22
			point10_x=x
			point10_y=y
			LABEL_TIME=1
		heading_twa=angle_limit(heading-twa-np.pi)
		goal_twa=goal_twa_limit
		heading_goal=goal_twa-heading_twa
		if abs(heading_goal)<np.pi/6:
			sa_goal=sa_goal_func(twa, goal_wind, heading)
		else:
			sa_goal=sa_goal_func(twa, goal_wind, heading)

	elif LABEL == 22:#左前执行延长5s
		LABEL_TIME += 1
		if LABEL_TIME>50:
			LABEL=0
		if abs(goal_wind)>np.pi/2:
			LABEL=0
		heading_twa=angle_limit(heading-twa-np.pi)
		goal_twa=goal_twa_limit
		heading_goal=goal_twa-heading_twa
		if abs(heading_goal)<np.pi/6:
			sa_goal=sa_goal_func(twa, goal_wind, heading)
		else:
			sa_goal=sa_goal_func(twa, goal_wind, heading)

	else:#正常航行
		if abs(goal_wind)<twa_limit:
			#if goal_wind>0:
			if angle_limit(heading-twa-np.pi)<0:
				LABEL=1
			else:
				LABEL=2
		heading_goal=heading_goal_func(goal_angle, heading,twa)

		if abs(heading_goal)<np.pi/6:
			sa_goal=sa_goal_func(twa, goal_wind, heading)
		else:
			sa_goal=sa_goal_func(twa, goal_wind, heading)

	print('goal_angle',goal_angle,'heading_goal',heading_goal,'LABEL',LABEL)
	heading_error.append(heading_goal)
	sa_output=sa_func(heading_error, sa_goal)
	sa_out.append(sa_output)
	ra_output=ra_func(heading_error,u)
	return sa_output,ra_output

def getOutMachPut(msg): #sailboat_message::Mach_msg
	mach_pub = Mach_msg()
	mach_pub.header.stamp = rospy.Time.now()
	mach_pub.header.frame_id = 'AHRS'
	#mach_pub.timestamp = rospy.Time.now()
	mach_pub.motor = 0
	mach_pub.rudder = msg[0]
	mach_pub.sail   = msg[1]
	mach_pub.PCCtrl = msg[2]
	return mach_pub

# rudder, sail, u, v, r, dheel, x, y, heading, heel,aws,awa
def getOutput(msg): #spare_function::spare_function_out
	out_pub = spare_function_out()
	out_pub.rudder = msg[0]
	out_pub.sail = msg[1]
	out_pub.u = msg[2]
	out_pub.v = msg[3]
	out_pub.r = msg[4]
	out_pub.dheel = msg[5]
	out_pub.x = msg[6]
	out_pub.y = msg[7]
	out_pub.heading = msg[8]
	out_pub.heel = msg[9]
	out_pub.aws = msg[10]
	out_pub.awa = msg[11]  
	return out_pub

def getOutParaPut(msg):#spare_function::spare_function_para
	para_pubmsg = spare_function_para()
	para_pubmsg.oyaw   = msg[1]
	para_pubmsg.rudderP= msg[2]
	para_pubmsg.rudderI= msg[3]
	para_pubmsg.rudderD= msg[4]
	para_pubmsg.sailP  = msg[5]
	para_pubmsg.sailI  = msg[6]
	para_pubmsg.sailD  = msg[7]
	return para_pubmsg

def sensorCallback(msg): #sailboat_message::Sensor_msg
	global sensor_submsg 
	sensor_submsg[0] = msg.ux
	sensor_submsg[1] = msg.vy
	sensor_submsg[2] = msg.gz
	sensor_submsg[3] = msg.gx
	sensor_submsg[4] = msg.Posx
	sensor_submsg[5] = msg.Posy
	sensor_submsg[6] = msg.Yaw
	sensor_submsg[7] = msg.Roll
	sensor_submsg[8] = msg.AWS
	sensor_submsg[9] = msg.AWA

def getConfigCallback(config, level): # spare_function::spare_function_Config
	global para_cfg
	if (config.PC_Ctrl == True):
		para_cfg[0] = 1
	else:
		para_cfg[0] = 0
	para_cfg[1] = config.oyaw
	para_cfg[2] = config.rudderP
	para_cfg[3] = config.rudderI
	para_cfg[4] = config.rudderD
	para_cfg[5] = config.sailP
	para_cfg[6] = config.sailI
	para_cfg[7] = config.sailD
	return config

if __name__ == "__main__":
	rospy.init_node("example", anonymous = True)

	mach_pub = rospy.Publisher('mach', Mach_msg, queue_size=5)
	spare_function_pub = rospy.Publisher('spare_function_out', spare_function_out, queue_size=5)
	spare_function_para_pub = rospy.Publisher('spare_function_para', spare_function_para, queue_size=5)
	
	rospy.Subscriber("sensor_kalman_msg", Sensor_msg, sensorCallback)
	config_srv = Server(spare_function_Config, getConfigCallback)

	rate = rospy.Rate(10)
	goal_list = [(20,0),(-55,70),(10,-10)]
	k = 0
	goal_fix = 0
	try:
		while not rospy.is_shutdown():
			sa, ra = module(sensor_submsg, goal_list[k])
			distance=math.sqrt((goal_list[k][0]-sensor_submsg[4])*(goal_list[k][0]-sensor_submsg[4])+(goal_list[k][1]-sensor_submsg[5])*(goal_list[k][1]-sensor_submsg[5]))
			if distance<3:
				k=k+1
			if k>len(goal_list)-1:
				k=0

			mach_np = [ra, sa, 1]
			# rudder, sail, u, v, tws, twa, x, y, heading, LABEL,aws,awa
			out_np = [ra*180/np.pi, sa*180/np.pi, sensor_submsg[0], sensor_submsg[1], tws, twa, sensor_submsg[4], sensor_submsg[5], sensor_submsg[6], LABEL, sensor_submsg[8], sensor_submsg[9]]
			# input : sensor_submsg 
			# cfg: para_cfg
			# output : mach_np out_np para_np

			mach_pubmsg = getOutMachPut(mach_np)
			out_pubmsg = getOutput(out_np)
			para_pubmsg = getOutParaPut(para_cfg)

			mach_pub.publish(mach_pubmsg)
			spare_function_pub.publish(out_pubmsg)
			spare_function_para_pub.publish(para_pubmsg)

			rate.sleep()
	except rospy.ROSInterruptException:
		pass
	#finally:
		#close()
	rospy.spin()
