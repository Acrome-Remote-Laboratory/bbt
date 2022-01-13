	import remotelab
import time

bbt = remotelab.BBT()

global param
		notify = False
		dof.enable = True
		err_prev = 0
		error_sum = 0
		interval = 0.004
		error_sum_x = 0
		error_sum_y = 0
		_filter_size = 10
		bbt_iter = 0
		x_mov_avg_filter = [0 for i in range(_filter_size)]
		y_mov_avg_filter = [0 for i in range(_filter_size)]

  
  feedforwardx = 725
					feedforwardy = 725
					windup_abs = 10
					calibration_x = (0,-50)
					calibration_y = (0,-8)

					setpointx = float(0)
					kpx = float(0.45)
					kix = float(0.001)
					kdx = float(0.2)

					setpointy = float(0)
					kpy = float(0.45)
					kiy = float(0.001)
					kdy = float(0.2)

					positionx, positiony = bbt.get_position()
					positionx = 1000 if positionx > 1000 else positionx
					positiony = 1000 if positiony > 1000 else positiony

					positionx += -100
					positiony += 0


					setpointx = (setpointx + 50) * 3 + 250
					setpointy = (setpointy + 60) * 3 + 250

					error = ((setpointx - positionx), (setpointy - positiony))
					error_sum_x += error[0]
					error_sum_y += error[1]

					#PID for X axis
					px = error[0] * kpx
					ix = error_sum_x*kix
					ix = (ix if -windup_abs <= ix <= windup_abs else ix/abs(ix)*windup_abs)

					x_mov_avg_filter[bbt_iter] = (error[0] - err_prev[0]) * kdx / interval
					dx = sum(x_mov_avg_filter[0:_filter_size])/_filter_size

					#PID for Y axis
					py = error[1] * kpy
					iy = error_sum_y*kiy
					iy = (iy if -windup_abs <= iy <= windup_abs else iy/abs(iy)*windup_abs)

					y_mov_avg_filter[bbt_iter] = (error[1] - err_prev[1]) * kdy / interval
					dy = sum(y_mov_avg_filter[0:_filter_size])/_filter_size

					bbt_iter = (bbt_iter + 1) % _filter_size

					outputx = feedforwardx + px + ix + dx
					outputy = feedforwardy + py + iy + dy

					outputx += calibration_x[0] * positionx + calibration_x[1]
					outputy += calibration_y[0] * positiony + calibration_y[1]

					#Lower limit for servo
					outputx = 500 if outputx <= 500 else outputx
					outputy = 500 if outputy <= 500 else outputy

					#Upper limit for servo
					outputx = 1000 if outputx >= 1000 else outputx
					outputy = 1000 if outputy >= 1000 else outputy

					####
					positionx = (positionx - 250) / 3 - 50
					positiony = (positiony - 250) / 3 - 60
					bbt.set_servo(int(outputx), int(outputy))
					err_prev = error