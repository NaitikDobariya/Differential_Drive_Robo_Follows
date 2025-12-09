import numpy as np
import math

def PID_path_follower(bot, desired_position, dt = 0.1, ang_setpoint = None):
    current_pos = bot.position
    current_angle = bot.angle
    error = desired_position - current_pos
    error_pos = np.linalg.norm(error)
    if ang_setpoint:
        desired_angle = ang_setpoint
    else:
        desired_angle = math.atan2(desired_position[1] - bot.position[1], desired_position[0] - bot.position[0])
    error_ang = desired_angle - current_angle
    error_ang = (error_ang + np.pi) % (2 * np.pi) - np.pi

    # error_ang = (error_ang + np.pi) % (2 * np.pi) - np.pi  # This ensures it's between -π and π
    acc_p = bot.kp_pos*error_pos
    ang_p = bot.kp_angular*error_ang

    if not bot.previous_pos_error or not bot.previous_ang_error:
         bot.previous_pos_error = error_pos
         bot.previous_ang_error = error_ang
    else:
         bot.previous_pos_error = error_pos
         bot.previous_ang_error = error_ang
         acc_p += bot.kd_pos*(error_pos - bot.previous_pos_error)/dt
         ang_p += bot.kd_angular*(error_ang - bot.previous_ang_error)/dt

    ang_p = np.clip(ang_p, -4, 4)
    acc_p = np.clip(acc_p, -10, 10)


    return acc_p , ang_p