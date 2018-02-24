from copy import copy

from Constants import *
from Dynamics import *

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

    def collision_incoming(self, pod: Pod):
        for inc_pod in self.pods.values():
            if inc_pod == pod:
                continue
            pos_a = eii(inc_pod.p, inc_pod.v)

    def calculate_point_speed(self, pod: Pod):
        next_current_angle = v_ang(self.map.get_next_check_p(pod.check_id) - self.map.get_check_p(pod.check_id))
        speed_on_next_point = v_max - (v_max - v_min)/math.pi * abs(next_current_angle - pod.a)
        if speed_on_next_point < v_min:
            speed_on_next_point = v_min

        # print(str(speed_on_point) + ' = '
        #       + str(v_max) + ' - ( ' + str(v_max) + ' - ' + str(v_min) + ')/math.pi * ' + str(
        #     abs(next_current_angle - pod.a)), file=sys.stderr)

        # print("speed_on_point: " + str(speed_on_point), file=sys.stderr)
        return speed_on_next_point

    def calculate_thrusts(self):
        self.calculate_thrust(self.pods[p1])
        self.calculate_thrust(self.pods[p2])

    def calculate_dests(self):
        self.calculate_dest(self.pods[p1])
        self.calculate_dest(self.pods[p2])

    def calculate_thrust(self, pod: Pod):
        # print("Calculating thrust", file=sys.stderr)
        self.slow_down = self.should_i_slowdown(self.calculate_point_speed(pod), pod)
        # print("Angle diff: " + str(self.player_pod.a_to_check * 180 / math.pi), file=sys.stderr)
        if self.slow_down or \
                (abs(ang(self.map.get_check_p(pod.check_id)-pod.p, pod.v)) * deg > 80
                 and norm(pod.v) > v_min):
            print("Slowing down", file=sys.stderr)
            pod.out_params[k_thrust] = 0
            pod.out_params[k_type] = k_thrust
        else:
            pod.out_params[k_thrust] = 100
            pod.out_params[k_type] = k_thrust
        if not pod.boost_used \
                and pod.out_params[k_thrust] == 100 \
                and abs(ang(self.map.get_check_p(pod.check_id)-pod.p, pod.v)) * deg < 10 \
                and abs(pod.a - v_ang(pod.v)) * deg < 10 \
                and norm(self.map.get_check_p(pod.check_id) - pod.p) > 5000:
            pod.out_params[k_type] = k_boost

    @staticmethod
    def get_speed_check_ang(to_point, pod: Pod):
        # print("check: " + str(self.map.get_current_check().p) +
        #       "- point: " + str(pod.p) +
        #       " speed: " + str(pod.v), file=sys.stderr)
        return ang(to_point - pod.p, pod.v)

    def calc_ang(self, pod: Pod):
        a_nc = ang_set(v_ang(self.map.get_check_p(pod.check_id+1) - self.map.get_check_p(pod.check_id)))
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

    def correct_dest(self, pod: Pod):
        pointing_to = copy(self.map.get_check_p(pod.check_id))
        beta = self.calc_ang(pod)
        print("beta: " + str(beta), file=sys.stderr)
        pointing_to[0] += 600*math.cos(beta)
        pointing_to[1] += 600*math.sin(beta)
        return pointing_to

    def calculate_dest(self, pod: Pod):
        # print("angle to check: " + str(self.get_speed_check_ang() * 180 / math.pi), file=sys.stderr)
        pointing_to = self.correct_dest(pod)
        print(str(pod.name) + ' check_id: ' + str(pod.check_id) + '->'
              + str(self.map.get_check_p(pod.check_id)), file=sys.stderr)
        # pointing_to = self.map.get_check_p(pod.check_id)
        if abs(self.get_speed_check_ang(pointing_to, pod) * deg) < 85:
            pod.out_params[k_pos] = rotate(pod.p, pointing_to, - self.get_speed_check_ang(pointing_to, pod))
        else:
            pod.out_params[k_pos] = pointing_to
        if self.slow_down:
            pod.out_params[k_pos] = self.map.get_check_p(pod.check_id + 1)

    def should_i_slowdown(self, v_d, pod: Pod):
        dist = np.linalg.norm(pod.p - self.map.get_check_p(pod.check_id))
        v_proj = np.dot(pod.v, u_vec(self.map.get_check_p(pod.check_id)-pod.p))
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
