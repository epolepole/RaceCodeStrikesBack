from copy import copy
import sys

from Dynamics import *
from Algorithms import *

from Pod import Pod
from Map import Map
from Check import Check


class Game:
    def __init__(self):
        self.map = Map()
        self.pods = {e1: Pod(e1),
                     e2: Pod(e2),
                     p1: Pod(p1),
                     p2: Pod(p2)}
        self.step = 0
        self.slow_down = False
        self.send_boost = False
        self.boost_used = False
        self.turning_started = False
        self.previous_dist = 0

    def initial_input(self):
        laps = int(input())
        check_count = int(input())
        self.map = Map(laps, check_count)
        for check_pos in range(check_count):
            check_x, check_y = [int(i) for i in input().split()]
            self.map.add_check(Check(check_x, check_y))

    def receive_input(self):
        self.pods[p1].update_status()
        self.pods[p2].update_status()
        self.pods[e1].update_status()
        self.pods[e2].update_status()

    def send_output(self):
        self.pods[p1].send_output()
        self.pods[p2].send_output()
        self.step += 1

    def calculate_thrusts(self):
        self.calculate_thrust(self.pods[p1])
        self.calculate_thrust(self.pods[p2])

    def calculate_dests(self):
        self.calculate_dest(self.pods[p1])
        self.calculate_dest(self.pods[p2])

    def calculate_thrust(self, pod: Pod):
        # print("Calculating thrust", file=sys.stderr)
        speed = calculate_point_speed(self.map.get_check(pod.check_id), self.map.get_next_check(pod.check_id), pod)
        self.slow_down = should_i_slowdown(self.map.get_check(pod.check_id), speed, pod)
        # print("Angle diff: " + str(self.player_pod.a_to_check * 180 / math.pi), file=sys.stderr)

        if slow_down(self.slow_down, self.map.get_check(pod.check_id), pod):
            print("Slowing down", file=sys.stderr)
            pod.out_params[k_thrust] = 0
            pod.out_params[k_type] = k_thrust
        else:
            pod.out_params[k_thrust] = 100
            pod.out_params[k_type] = k_thrust
        if use_boost(self.map.get_check(pod.check_id), pod):
            pod.out_params[k_type] = k_boost

    def calculate_dest(self, pod: Pod):
        # print("angle to check: " + str(self.get_speed_check_ang() * 180 / math.pi), file=sys.stderr)
        pointing_to = correct_dest(self.map.get_check(pod.check_id), self.map.get_check(pod.check_id + 1), pod)
        print(str(pod.name) + ' check_id: ' + str(pod.check_id) + '->'
              + str(self.map.get_check_p(pod.check_id)), file=sys.stderr)
        # pointing_to = self.map.get_check_p(pod.check_id)
        if abs(get_speed_check_ang(pointing_to, pod) * deg) < 85:
            pod.out_params[k_pos] = rotate(pod.p, pointing_to, - get_speed_check_ang(pointing_to, pod))
        else:
            pod.out_params[k_pos] = pointing_to
        if self.slow_down:
            pod.out_params[k_pos] = self.map.get_check_p(pod.check_id + 1)
