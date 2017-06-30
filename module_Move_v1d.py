#!/usr/bin/env python
# module_Move_v1a.py   #  module with MoveAlongWall,  MoveFwd_On_Bearing,  MoveBck functions
# module_Move_v1b.py   #  added stop if no wall to follow, added code for move along right side
# module_Move_v1b.py   #  added two reading average for fr & fl to MoveFwd 
# module_Move_v1c.py   #  changed MoveAlongWall to require 2 or more no wall readings before stopping
# module_Move_v1d.py   #  added track xy  added MoveBck_Update_xy

import time

#    import sensor & pin assignment modules  ========================================================================
import module_compass_v1a
get_bearing=module_compass_v1a.get_bearing
import module_measure_distance_v1a   
measure_distance=module_measure_distance_v1a.measure_distance
from module_pin_motor_setup_v01a import *
import module_trackxy_v01a 
calc_delta_xy=module_trackxy_v01a.calc_delta_xy


# ===========================================================================================================
#  Movement Functions MoveFwd, MoveFwd_bi,  MoveAlongWall,  MoveBck
# ===========================================================================================================
def MoveFwd(m1speed,m2speed,maxdist):  # moves car forward Motor 1 speed & Motor 2 speed
	print "run forwards"
	cum_counter_r=0 # initiate cummulative counter
	cum_counter_l=0	
	cum_counter_inc=4  # used for taking a sensor reading every x ODO readings
	mp1 = GPIO.PWM(Motor1Enable, 1000)     # set Frequece to 1KHz
	mp2 = GPIO.PWM(Motor2Enable, 1000)    
	mp1.start(m1speed)                     # Set initial duty cycle (motor speed)
	mp2.start(m2speed)
	new_time=time.time()              # initialize time
	wall_distance_fc = 1000  # initialize wall distances
	wall_distance_fl = 1000
	wall_distance_fr = 1000

		
	while wall_distance_fc>30 and wall_distance_fr>10 and wall_distance_fl>10 and cum_counter_r<maxdist:	
		GPIO.output(Motor1Pin1, GPIO.HIGH)  # clockwise
		GPIO.output(Motor1Pin2, GPIO.LOW)
		GPIO.output(Motor2Pin1, GPIO.HIGH)  # clockwise
		GPIO.output(Motor2Pin2, GPIO.LOW)		
		# look for sensor events and increment counter
		if GPIO.event_detected(odo_sensor_r):
			cum_counter_r=cum_counter_r+1
			if cum_counter_r%cum_counter_inc==0:	
				old_time=new_time
				new_time=time.time()
				delta_time=round(new_time-old_time,3)
				speed=round(1/delta_time*cum_counter_inc,0)
				if speed >100:   # limit speed to 100
					speed=100 
				#print "speed: ",speed, "nt ",new_time,"dt ",delta_time,"ODO",cum_counter_r
				new_bearing=  round(get_bearing(),0)
				wall_distance_fc = measure_distance(TRIG_FC,ECHO_FC) # update distances
				wall_distance_fl1 = round(measure_distance(TRIG_FL,ECHO_FL),0)
				wall_distance_fr1 = round(measure_distance(TRIG_FR,ECHO_FR),0)
				time.sleep(0.2)
				wall_distance_fl2 = round(measure_distance(TRIG_FL,ECHO_FL),0)
				wall_distance_fr2 = round(measure_distance(TRIG_FR,ECHO_FR),0)
				wall_distance_fl=(wall_distance_fl1+wall_distance_fl2)/2
				wall_distance_fr=(wall_distance_fr1+wall_distance_fr2)/2				
				print "cum_ODO,dt,s,b-",cum_counter_r,delta_time,speed,new_bearing,"d fc fl fr--",wall_distance_fc,wall_distance_fl,wall_distance_fr
				
		
	# stop
	mp1.stop  # stops PWM
	mp2.stop
	GPIO.output(Motor1Enable, GPIO.LOW) # motor stop
	GPIO.output(Motor2Enable, GPIO.LOW) # added code motor2 stop
# ===========================================================================================================		
def MoveFwd_bi(m1speed,m2speed,maxdist,speed_input,bearing_input,start_x,start_y):  # move car forward along bearing input
	print "run forwards along bearing",bearing_input
	global MoveFwd_bi_stop   # used to export reason for stopping
	global MoveFwd_bi_end_x   # used to export final x location
	global MoveFwd_bi_end_y
	x=start_x
	y=start_y
	time.sleep(0.2)  # wait for distance sensor to settle
	cum_counter_r=0 # initiate cummulative counter
	old_cum_counter_r=0
	cum_counter_l=0
	bearing_adjust_factor=0.3  # this times bearing change = +- m1/2 speed adjustment 
	speed_adjust_factor= 0.1  # same as bearing_af but for speed control
	min_PWM=15
	front_c_speed_adj_dist=90   # front center distance for speed reduction
	front_lr_speed_adj_dist=70   # front left or right distance for speed reduction
	fc_stop =30   # distances for stopping car
	fl_stop =30
	fr_stop=30
	min_input_speed=5
	new_time=time.time()  # intialize new_time for speed calc
	past_360_flag=0   # used to track crossing the 0/360 angle
	new_bearing=get_bearing()
	wall_distance_fc = measure_distance(TRIG_FC,ECHO_FC) # check front distance before start
	wall_distance_fl1 = measure_distance(TRIG_FL,ECHO_FL)  # take 2 readings and ave
	time.sleep(0.2)
	wall_distance_fl2 = measure_distance(TRIG_FL,ECHO_FL)
	wall_distance_fl=(wall_distance_fl1+wall_distance_fl2)/2
	wall_distance_fr1 = measure_distance(TRIG_FR,ECHO_FR)  # take 2 readings and ave
	time.sleep(0.2)
	wall_distance_fr2 = measure_distance(TRIG_FR,ECHO_FR)
	wall_distance_fr=(wall_distance_fr1+wall_distance_fr2)/2	
		
	print "d fl,fc,fr",wall_distance_fl,wall_distance_fc,wall_distance_fr
	mp1 = GPIO.PWM(Motor1Enable, 1000)     # set Frequece to 1KHz
	mp2 = GPIO.PWM(Motor2Enable, 1000)    
	mp1.start(m1speed)                     # Set initial duty cycle (motor speed)
	mp2.start(m2speed)

	while wall_distance_fc>fc_stop and wall_distance_fr>fr_stop and wall_distance_fl>fl_stop and cum_counter_r<maxdist:	
		speed=30  # intialize speed
		# Run motor clockwise		
		#GPIO.output(MotorEnable, GPIO.HIGH) # replaced with PWM
		#GPIO.output(Motor2Enable, GPIO.HIGH) #replaced with PWM
		GPIO.output(Motor1Pin1, GPIO.HIGH)  # clockwise
		GPIO.output(Motor1Pin2, GPIO.LOW)
		GPIO.output(Motor2Pin1, GPIO.HIGH)  # clockwise
		GPIO.output(Motor2Pin2, GPIO.LOW)		
		# look for sensor events and increment counter
		if GPIO.event_detected(odo_sensor_r):
			cum_counter_r=cum_counter_r+1
			cum_counter_inc=4
			if cum_counter_r%cum_counter_inc==0:	
				old_time=new_time
				new_time=time.time()
				delta_time=new_time-old_time
				speed=round(1/delta_time*cum_counter_inc,0)
				if speed >100:   # limit speed to 100
					speed=100 
				# print "speed: ",speed, "nt ",new_time,"dt ",delta_time,"ODO",cum_counter_r

				if abs(speed-speed_input)>2:
					m1speed=m1speed-(speed-speed_input)*speed_adjust_factor
					m2speed=m2speed-(speed-speed_input)*speed_adjust_factor
					#------ fix m1/2speed if out of range
					if m1speed >100:
						m1speed=100
					if m1speed<min_PWM:
						m1speed=min_PWM
					if m2speed >100:
						m2speed=100
					if m2speed<min_PWM:
						m2speed=min_PWM		
					mp1.ChangeDutyCycle(m1speed)                     # update motor speed
					mp2.ChangeDutyCycle(m2speed)
					# print "M1/2Speed ",m1speed," ",m2speed
				old_bearing=new_bearing	
				new_bearing=  round(get_bearing(),0)
				if new_bearing-old_bearing>90:   # looks for angles crossing 360 line & adds/subtracts 360
					new_bearing=new_bearing-360
				if new_bearing-old_bearing<-90:
					new_bearing=new_bearing+360
				#  Update x y position -----------------------------------------------------------------------------
				ave_bearing=(old_bearing+new_bearing)/2
				delta_ODO=cum_counter_r-old_cum_counter_r
				old_cum_counter_r=cum_counter_r  # reset old cum counter value
				delta_xy=calc_delta_xy(delta_ODO,ave_bearing)
				x=delta_xy[0]+x
				y=delta_xy[1]+y
				print "x,y",x,y

				if abs(new_bearing-bearing_input)>2: # update m1/2 speed if bearing changes over x
					m1speed=m1speed+(new_bearing-bearing_input)*bearing_adjust_factor
					m2speed=m2speed-(new_bearing-bearing_input)*bearing_adjust_factor
					#------ fix m1/2speed if out of range
					if m1speed >100:
						m1speed=100
					if m1speed<min_PWM:
						m1speed=min_PWM
					if m2speed >100:
						m2speed=100
					if m2speed<min_PWM:
						m2speed=min_PWM		
					mp1.ChangeDutyCycle(m1speed)                     # update motor speed
					mp2.ChangeDutyCycle(m2speed)
					# print "M1/2Speed bearing ",m1speed," ",m2speed
				wall_distance_fc = measure_distance(TRIG_FC,ECHO_FC) # update distances
				
				
				wall_distance_fl_old = wall_distance_fl    # assign last ave to old 
				wall_distance_fr_old = wall_distance_fr
				wall_distance_fl_new = round(measure_distance(TRIG_FL,ECHO_FL),0)
				wall_distance_fr_new = round(measure_distance(TRIG_FR,ECHO_FR),0)
				wall_distance_fl=round((wall_distance_fl_new+wall_distance_fl_old)/2,0)  # use average of last two readings			
				wall_distance_fr=round((wall_distance_fr_new+wall_distance_fr_old)/2,0)	
				#  print "s,b,dist_fc,fl,fr",speed,new_bearing,wall_distance_fc ,wall_distance_fl,wall_distance_fr,"M1/2",m1speed,"/",m2speed
				wall_dist_lr_min= min(wall_distance_fl,wall_distance_fr)
				if wall_dist_lr_min < front_lr_speed_adj_dist:  # slow down if front l/r sensors see wall 
					speed_input=speed_input*(wall_dist_lr_min /front_lr_speed_adj_dist)  # adjust speed using minimum wall dist
					if speed_input< min_input_speed:  # dont decrease speed below min
						speed_input=min_input_speed
				if wall_distance_fc < front_c_speed_adj_dist:  # slow down if front center sensor sees wall 
					speed_input=speed_input*(wall_distance_fc /front_c_speed_adj_dist)  # adjust speed using minimum wall dist
					if speed_input< min_input_speed:  # dont decrease speed below min
						speed_input=min_input_speed

		if GPIO.event_detected(odo_sensor_l):  # increments odo_left cum counter 
			cum_counter_l=cum_counter_l+1
	mp1.ChangeDutyCycle(0)                     # set speed to zero
	mp2.ChangeDutyCycle(0)

	#  report out the reason for stopping
	if wall_distance_fc<=fc_stop:
		MoveFwd_bi_stop="fc"
	elif wall_distance_fl<=fl_stop:
		MoveFwd_bi_stop="fl"
	elif wall_distance_fr<=fr_stop:
		MoveFwd_bi_stop="fr"
	else: 
		MoveFwd_bi_stop="ODO"
	print "MoveFwd_bi_stop=",MoveFwd_bi_stop
	#  report out final xy 
	MoveFwd_bi_end_x=x
	MoveFwd_bi_end_y=y
	# MoveFwd_bi_end_xy[1]=y
	print "final xy",x,y	
	# stop
	mp1.stop  # stops PWM
	mp2.stop
	GPIO.output(Motor1Enable, GPIO.LOW) # motor stop
	GPIO.output(Motor2Enable, GPIO.LOW) # added code motor2 stop	
# ===========================================================================================================
def MoveAlongWall(m1_dc_start,m2_dc_start,maxdist,wall_side,wall_standoff):  
	print "run along wall with ",wall_standoff,"standoff"
	# Initialize variables -----------------------------------------------------------------------------------------------------------------------------------------------------
	global MoveAlong_stop  #  used to output the sensor that caused car to  stop fc,fl, fr,ODO or no_sw
	cum_counter_inc1=3  # set the interval (# of ODO counts)  between fl/fr distance sensor readings  (extra readings  fl & fr needed due to random bad readings)
	cum_counter_inc2=2*cum_counter_inc1  # set the interval (# of ODO counts)  bearing and fc bc distance reading and also motor adjustments
	cum_counter_r=0 # initiate cummulative counter
	cum_counter_l=0
	max_wall_SO_error=20.001
	max_side_wall_dist_last_delta=5.0001
	m12_ave=(m1_dc_start+m2_dc_start)/2
	m_dc_max=99.001  # maximim value for motor control duty cycle
	m_dc_min=15.0001    #  minimim value for motor control duty cycle 
	front_c_speed_adj_dist=50.0001
	m12_ave_min=25.0001
	m_ratio_max=3.0001
	fc_stop=30     # car stops if front  center dist is less than this
	fl_stop=5        # same for front left distance 
	fr_stop=5		
	no_sw_stop=90    # triggors no_sw_counter 
	no_sw_counter=0
	max_no_sw_counter=2    # stops car if counter reaches this value
	# take initial bearing and wall distance readings and print results ------------------------------------------------------------------------------------------------	
	time.sleep(0.2)  # wait for sensors to settle
	new_bearing= 100 # get_bearing()   temp debug
	wall_distance_fc =  measure_distance(TRIG_FC,ECHO_FC) # check front distance before start
	wall_distance_fl1 = measure_distance(TRIG_FL,ECHO_FL)  # take 2 readings and ave
	time.sleep(0.2)
	wall_distance_fl2 = measure_distance(TRIG_FL,ECHO_FL)
	wall_distance_fl= (wall_distance_fl1+wall_distance_fl2)/2
	wall_distance_fr1 =measure_distance(TRIG_FR,ECHO_FR)  # take 2 readings and ave
	time.sleep(0.2)
	wall_distance_fr2 = measure_distance(TRIG_FR,ECHO_FR)
	wall_distance_fr=(wall_distance_fr1+wall_distance_fr2)/2	
	if wall_side=="left":
		side_wall_dist=wall_distance_fl
	else:
		side_wall_dist=wall_distance_fr
	print "initial readings: b-", new_bearing,"d fl,fc,fr,swd-",wall_distance_fl,wall_distance_fc,wall_distance_fr,side_wall_dist
	
	# set up motor inputs & initialize time for dt  -------------------------------------------------------------------------------------------------------------------
	mp1 = GPIO.PWM(Motor1Enable, 1000)     # set Frequece to 1KHz
	mp2 = GPIO.PWM(Motor2Enable, 1000)    
	mp1.start(50)         # Set initial duty cycle (motor speed)
	mp2.start(46)
	new_time=time.time()  # initialize new_time
	#  Start loop for moving along wall-------------------------------------------------------------------------------------------------------------------------------
	while wall_distance_fc>fc_stop and wall_distance_fr>fr_stop and wall_distance_fl>fl_stop and no_sw_counter< max_no_sw_counter and cum_counter_r<maxdist:	 
		# start motors -----------------------------------------------------------------
		if m1_dc_start >=0:    # run m1 (left side) forward if positive
			GPIO.output(Motor1Pin1, GPIO.HIGH)
			GPIO.output(Motor1Pin2, GPIO.LOW)
		else:   # run m1 backward if negative
			GPIO.output(Motor1Pin2, GPIO.HIGH)
			GPIO.output(Motor1Pin1, GPIO.LOW)		
		if m2_dc_start >=0:    # run m2 (right side) forward if positive	
			GPIO.output(Motor2Pin1, GPIO.HIGH)
			GPIO.output(Motor2Pin2, GPIO.LOW)		
		else:   # run backward
			GPIO.output(Motor2Pin2, GPIO.HIGH)
			GPIO.output(Motor2Pin1, GPIO.LOW)			
		# Look for right ODO events ---------------------------------------------------------------------------------------------------------------------------------------
		if GPIO.event_detected(odo_sensor_r):
			cum_counter_r=cum_counter_r+1	
			if cum_counter_r%cum_counter_inc1==0:				
				#  Get 1st fl and fr distance sensor readings ( extra readings for fl & fr to mitigate the random bad readings) 
				wall_distance_fl_1 = round(measure_distance(TRIG_FL,ECHO_FL),0)
				wall_distance_fr_1 = round(measure_distance(TRIG_FR,ECHO_FR),0)
			if cum_counter_r%cum_counter_inc2==0:	
				#  Calc speed in ODO counts per second  also = cm /sec -----------------------------
				old_time=new_time
				new_time=time.time()
				delta_time=new_time-old_time
				speed=round(1/delta_time*cum_counter_inc2,0)
				if speed >100:   # limit speed to 100
					speed=100 
				#  Get new bearing and wall distances  -----------------------------------------------------
				old_bearing=new_bearing	
				new_bearing=  round(get_bearing(),0)
				wall_distance_fc = measure_distance(TRIG_FC,ECHO_FC) # update distances
				wall_distance_fl_2 = round(measure_distance(TRIG_FL,ECHO_FL),0)
				wall_distance_fr_2 = round(measure_distance(TRIG_FR,ECHO_FR),0)
				wall_distance_fl=round((wall_distance_fl_1+wall_distance_fl_2)/2,0)  # use average of last two readings			
				wall_distance_fr=round((wall_distance_fr_1+wall_distance_fr_2)/2,0)
				side_wall_dist_last=side_wall_dist   # update wall dist old 
				#  Calc the side_wall_distance using the wall side sensor   -----------------
				if wall_side=="left":   
					side_wall_dist=wall_distance_fl  #  set proper side wall input 
				else:
					side_wall_dist=wall_distance_fr
				#  Check if side_wall_dist is > no_sw_stop  if so increment no_sw_counter  if not reset counter  -------------------
				if side_wall_dist>=no_sw_stop:
					no_sw_counter=no_sw_counter+1
				else:
					no_sw_counter=0
				#  Calc the side_wall_dist_last_delta.  Limit value to max  -------------------					
				side_wall_dist_last_delta=side_wall_dist-side_wall_dist_last
				if side_wall_dist_last_delta>=0:   # limit side_wall_dist_last_delta to max value
					side_wall_dist_last_delta= min(max_side_wall_dist_last_delta,side_wall_dist_last_delta)
				else: 
					side_wall_dist_last_delta= max(-max_side_wall_dist_last_delta,side_wall_dist_last_delta)
				#  now calc wall_StandOff_error.  Limit error to max value  ----------------------------------
				if side_wall_dist-wall_standoff>=0:
					wall_SO_error= min(side_wall_dist-wall_standoff,max_wall_SO_error)
				else: 
					wall_SO_error= max(side_wall_dist-wall_standoff,-max_wall_SO_error)		
				# Calc pct steer based on the last delta and standoff error.. pct steer is like a steering wheel position  + = turn left  /  - = turn right
				if wall_side =="left":
					m_pct_steer=wall_SO_error/max_wall_SO_error +side_wall_dist_last_delta/max_side_wall_dist_last_delta
				else: 
					m_pct_steer=(wall_SO_error/max_wall_SO_error +side_wall_dist_last_delta/max_side_wall_dist_last_delta)*-1						
				#  now adjust motor speeds based on % steer & m_ratio_max       m_ratio_max set how sharp  a turn when pct steer =100%
				if m_pct_steer<=0:    # turn left           (pct steer : + = turn left  /  - = turn right)					
					m_ratio=1/(1+(m_ratio_max*2-1)*abs(m_pct_steer))
					m2_dc=m12_ave*2*m_ratio/(1+m_ratio)
					m1_dc=m2_dc/(m_ratio)
				else:		# turn right
					m_ratio=1+(m_ratio_max-1)*m_pct_steer
					m2_dc=m12_ave*2*m_ratio/(1+m_ratio)
					m1_dc=m2_dc/m_ratio
									
				#  limit m1 m2 values to min and max & update motor duty cycles---------------
				if m1_dc >m_dc_max:
					m1_dc=m_dc_max
				if m1_dc<m_dc_min:
					m1_dc=m_dc_min
				if m2_dc >m_dc_max:
					m2_dc=m_dc_max
				if m2_dc<m_dc_min:
					m2_dc=m_dc_min
				mp1.ChangeDutyCycle(m1_dc)  # update motor duty cycles
				mp2.ChangeDutyCycle(m2_dc)
				print "s,b,dist_fc,fl,fr,so_e",speed,"b-",new_bearing,"d-",wall_distance_fc ,wall_distance_fl,wall_distance_fr,"  SOE",wall_SO_error,"M1/2",m1_dc,"/",m2_dc
				
				if wall_distance_fc < front_c_speed_adj_dist:  # slow down if front center sensor sees wall 
					m12_ave = m12_ave_min

		if GPIO.event_detected(odo_sensor_l):  # increments odo_left cum counter 
			cum_counter_l=cum_counter_l+1
	# stop   ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	mp1.ChangeDutyCycle(0)                     # set speed to zero
	mp2.ChangeDutyCycle(0)		

	mp1.stop  # stops PWM
	mp2.stop
	GPIO.output(Motor1Enable, GPIO.LOW) # motor stop
	GPIO.output(Motor2Enable, GPIO.LOW) 
	#  use if statements to determine what stopped the car and record in global variable MoveAlong_stop -----------------------------------------------------------------------------------
	if wall_distance_fc<=fc_stop:
		MoveAlong_stop="fc"
	elif wall_distance_fl<=fl_stop:
		MoveAlong_stop="fl"
	elif wall_distance_fr<=fr_stop:
		MoveAlong_stop="fr"
	elif side_wall_dist>=no_sw_stop:
		MoveAlong_stop="no_sw"
	else: 
		MoveAlong_stop="ODO"
	print "MoveAlong_stop=",MoveAlong_stop
	
# ===========================================================================================================================
def MoveBck(m1speed,m2speed,maxdist):  # moves car forward Motor 1 speed & Motor 2 speed
	print "run backwards"
	time.sleep(0.2)  # wait for distance sensor to settle
	cum_counter_r=0 # initiate cummulative counter
	cum_counter_l=0 
	# move back x number of sensor pulses	
	wall_distance_bc = measure_distance(TRIG_BC,ECHO_BC) # check distance before start
	print "Distance : %.1f" % wall_distance_bc
	mp1 = GPIO.PWM(Motor1Enable, 1000)     # set Frequece to 1KHz
	mp2 = GPIO.PWM(Motor2Enable, 1000)    
	mp1.start(m1speed)                     # Set initial duty cycle (motor speed)
	mp2.start(m2speed)
	new_time=time.time()

	while wall_distance_bc>50 and cum_counter_r<maxdist:		
		mp1.ChangeDutyCycle(m1speed)      # reset PWM to input speed
		mp2.ChangeDutyCycle(m2speed)
		GPIO.output(Motor1Pin2, GPIO.HIGH)  # anti-clockwise
		GPIO.output(Motor1Pin1, GPIO.LOW)
		GPIO.output(Motor2Pin2, GPIO.HIGH)  # anti-clockwise
		GPIO.output(Motor2Pin1, GPIO.LOW)		
		# look for sensor events and increment counter
		if GPIO.event_detected(odo_sensor_r):
			cum_counter_r=cum_counter_r+1
			if cum_counter_r%5==0:
				mp1.ChangeDutyCycle(0)      # set PWM to zero  stop to take readings
				mp2.ChangeDutyCycle(0)
				old_time=new_time
				new_time=time.time()
				dt=new_time-old_time
				wall_distance_bc = measure_distance(TRIG_BC,ECHO_BC)
				wall_distance_fc = measure_distance(TRIG_FC,ECHO_FC)
				current_bearing=get_bearing() 
				print "dt",dt, "odo r/l ",cum_counter_r,"/",cum_counter_l,"wall dist b/f",wall_distance_bc,wall_distance_fc
		
#		if GPIO.event_detected(odo_sensor_l):
#			cum_counter_l=cum_counter_l+1
	# stop
	mp1.stop  # stops PWM
	mp2.stop
	GPIO.output(Motor1Enable, GPIO.LOW) # motor stop
	GPIO.output(Motor2Enable, GPIO.LOW) # added code motor2 stop	
# ===========================================================================================================================
def MoveBck_Update_xy(movebck_dist,start_x,start_y):
	time.sleep(0.2)  # time for sensor to settle
	current_bearing=get_bearing()   # update current bearing
	movebck_dist=10
	MoveBck(40,40,movebck_dist)   #  (m1speed,m2speed,maxdist)		
	old_bearing=current_bearing
	time.sleep(0.2)  # time for sensor to settle
	current_bearing=get_bearing()   # update current bearing		
	ave_bearing=(old_bearing+current_bearing)/2
	delta_xy=calc_delta_xy(movebck_dist,ave_bearing)
	MoveBck_end_xy=[start_x+delta_xy[0],start_y+delta_xy[1]]
	return MoveBck_end_xy
