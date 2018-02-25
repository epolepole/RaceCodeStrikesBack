from copy import copy
import sys

from Dynamics import *

from Pod import Pod
from Check import Check


def calculate_point_speed(check: Check, check_next: Check, pod: Pod):
    next_current_angle = vector_ang(check_next.p - check.p)
    speed_on_next_point = v_max - (v_max - v_min) / math.pi * abs(next_current_angle - pod.a)
    if speed_on_next_point < v_min:
        speed_on_next_point = v_min
    # print(str(speed_on_point) + ' = '
    #       + str(v_max) + ' - ( ' + str(v_max) + ' - ' + str(v_min) + ')/math.pi * ' + str(
    #     abs(next_current_angle - pod.a)), file=sys.stderr)

    # print("speed_on_point: " + str(speed_on_point), file=sys.stderr)
    return speed_on_next_point


def should_i_slowdown(check: Check, v_d, pod: Pod):
    dist = np.linalg.norm(pod.p - check.p)
    v_proj = np.dot(pod.v, u_vec(check.p - pod.p))
    # print("dist: " + str(dist), file=sys.stderr)
    # print("v_proj: " + str(v_proj), file=sys.stderr)
    # t = t_e(dist - 600, v_proj)
    info = iterate_until_dist(dist - 600, v_proj)
    # info = {'t':0, 'a':0, 'e':0, 'v':0, }
    # print("t_point: " + str(info['t']), file=sys.stderr)
    # print("a_point: " + str(info['a']), file=sys.stderr)
    # print("e_point: " + str(info['e']), file=sys.stderr)
    # print("v_point: " + str(info['v']), file=sys.stderr)
    if pod.turning_started:
        return True
    elif info['v'] > v_d:
        pod.turning_started = True
        return True
    return False


def correct_dest(check: Check, check_next: Check, pod: Pod):
    pointing_to = copy(check.p)
    beta = calc_ang(check, check_next, pod)
    print("beta: " + str(beta), file=sys.stderr)
    pointing_to[0] += 600*math.cos(beta)
    pointing_to[1] += 600*math.sin(beta)
    return pointing_to


def calc_ang(check: Check, check_next: Check, pod: Pod):
    a_nc = ang_set(vector_ang(check_next.p - check.p))
    a_p = ang_set(pod.a)
    if a_nc > a_p:
        med = ang_set((a_nc - a_p)/2)
        beta = a_p - med
    else:
        med = ang_set((a_p - a_nc)/2)
        beta = a_nc - med

    print('aNC:' + str(a_nc*deg) + ' a_p: ' + str(a_p*deg) + ' med: ' + str(med*deg) + ' beta = ' + str(beta*deg),
          file=sys.stderr)
    return ang_set(beta)


def get_speed_check_ang(to_point, pod: Pod):
    # print("check: " + str(self.map.get_current_check().p) +
    #       "- point: " + str(pod.p) +
    #       " speed: " + str(pod.v), file=sys.stderr)
    return ang(to_point - pod.p, pod.v)


def use_boost(check: Check, pod: Pod):
    return not pod.boost_used \
           and pod.out_params[k_thrust] == 100 \
           and abs(ang(check.p - pod.p, pod.v)) * deg < 10 \
           and abs(pod.a - vector_ang(pod.v)) * deg < 10 \
           and norm(check.p - pod.p) > 5000


def slow_down(slowing: bool, check: Check, pod: Pod):
    return slowing or \
        (abs(ang(check.p - pod.p, pod.v)) * deg > 80
            and norm(pod.v) > v_min)


def collision_incoming(self, pod: Pod):
    for inc_pod in self.pods.values():
        if inc_pod == pod:
            continue
        pos_a = eii(inc_pod.p, inc_pod.v)


def calculate_outer_point(check_0: Check, check_1: Check, current_pod: Pod):
    return ''