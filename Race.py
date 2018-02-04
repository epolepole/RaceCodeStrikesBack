import sys
import math
import numpy as np
from numpy.linalg import norm
from copy import copy

# Auto-generated code below aims at helping you parse
# the standard input according to the problem statement.

ka = 0.85/0.21
dt = 1
speed_on_point = 200
v_max = 500
v_min = 170
deg = 180/math.pi
k_check = 'check'
k_pos = 'pos'
k_boost = 'BOOST'
k_shield = 'SHIELD'
k_type = 'type'
k_thrust = 'thrust'

def rotate(origin:np.array, point:np.array, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    # print("rotating  " + str(point) + " through " + str(origin) + " into " + str(np.array([qx, qy])), file=sys.stderr)
    return np.array([qx, qy])


def a(v0, thrust=0):
    return 0.85*thrust - 0.21*v0


def v_t(v0, t, thrust=0):
    a = ka * thrust
    b = 0.21
    return a + (v0 + a)*math.exp(-b*t)


def e_t(v0, t, e0=0, thrust=0):
    a = ka*thrust
    b = v0 - ka*thrust
    c = 0.21
    return e0 + b/c + a*t - b/c*math.exp(-c*t)


def t_e(e, v0, thrust=0):
    t = 0
    esp = e_t(0, v0, thrust, t)

    print("distance = " + str(e), file=sys.stderr)
    print("e("
          + str(v0) + ','
          + str(t) + ','
          + '0,'
          + str(thrust) + ") = " + str(e_t(v0, t, 0, thrust)), file=sys.stderr)

    while esp < e and t < 100:
        # print("e(" + str(t) + ") = " + str(esp), file=sys.stderr)
        t = t + 1
        esp = e_t(0, v0, thrust, t)

    return t


def ai(v_i, thrust = 0):
    return 0.85*thrust - 0.21*v_i


def vii(v_i, a_i):
    return v_i + a_i*dt


def eii(e_i, v_i):
    return e_i + v_i*dt


def u_vec(vector):
    """ Returns the unit vector of the vector.  """
    if norm(vector) == 0: return np.array([0,0])
    return vector / norm(vector)


def ang(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'
    """
    # print("ang v2: " +str(v_ang(v2)) + " - ang v1: " +str(v_ang(v1)) , file=sys.stderr)
    return v_ang(v2) - v_ang(v1)


def v_ang(v):
    """ Returns the angle in radians between vectors 'v1' and 'v2'
    """
    v_u = u_vec(v)
    return np.arctan2(v_u[1], v_u[0])


def ang_set(ang):
    return ang % 2*math.pi


class Check:
    def __init__(self, i_x=-1, i_y=-1):
        self.p = np.array([i_x, i_y])

    def __eq__(self, other: np.array):
        return np.array_equal(self.p, other.p)

    def x(self):
        return self.p[0]

    def y(self):
        return self.p[1]


class Pod:
    def __init__(self, name, i_x=-1, i_y=-1):
        self.name=name
        self.p = np.array([i_x, i_y])
        self.p_prev = np.array([i_x, i_y])
        self.v = np.array([0, 0])
        self.v_ang = v_ang(self.v)
        self.v_prev = np.array([0, 0])
        self.v_prev_ang = v_ang(self.v_prev)
        self.a = 0
        self.a_prev = 0
        self.check_id = -1
        self.out_params = {}

        self.turning_started = False
        self.boost_used = False

    def add_step(self, i_p: np.array, i_v: np.array, i_a: int):
        print("Pod " + str(self.name) + ' i_p: ' + str(i_p) + ' i_v: ' + str(i_v), file=sys.stderr)
        self.p_prev = self.p
        self.p = i_p
        self.v_prev = self.v
        self.v_prev_ang = self.v_ang
        self.v = i_v
        self.v_ang = v_ang(self.v)
        self.a_prev = self.a
        self.a = i_a
        # print("Angle deviation: " + str((v_ang(self.v) - self.a) * 180 / math.pi), file=sys.stderr)

    # def set_angle(self, next_check: Check, angle):
    #     self.a_to_check = angle * math.pi/180
    #     self.a_prev = self.a
    #     self.a = self.a_to_check + v_ang(next_check.p - self.p)
    #     print("input angle: " + str(angle), file=sys.stderr)
    #     print("pod speed angle: " + str(v_ang(self.p) * 180 / math.pi), file=sys.stderr)
    #     print("pod possition angle: " + str(v_ang(next_check.p) * 180 / math.pi), file=sys.stderr)
    #     print("pod to check angle: " + str(v_ang(next_check.p - self.p) * 180 / math.pi), file=sys.stderr)
    #     print("pod orientation angl: " + str(self.a * 180 / math.pi), file=sys.stderr)

    def collision(self):
        #print('p: ' + str(self.p) + ' p_prev: ' + str(self.p_prev), file=sys.stderr)
        collision = abs(ang(self.v, self.p - self.p_prev) * deg) > 2
        if collision:
            print("collision detected on pod " + str(self.name), file=sys.stderr)
        return collision

    def update_status(self):
        x, y, vx, vy, angle, next_check_id = [int(i) for i in input().split()]
        print("check_id in input " + str(next_check_id), file=sys.stderr)
        if self.check_id != next_check_id:
            self.turning_started = False
            self.check_id = next_check_id
        self.add_step(np.array([x,y]), np.array([vx, vy]), angle/deg)
        if self.collision():
            self.turning_started = False

    def send_output(self):
        if self.out_params[k_type] == k_boost:
            print("BOOSTING", file=sys.stderr)
            self.boost_used = True
            print(str(int(self.out_params[k_pos][0])) + " " + str(int(self.out_params[k_pos][1])) + " " + k_boost+ " " + k_boost)
        elif self.out_params[k_type] == k_shield:
            print("SHIELDING", file=sys.stderr)
            print(str(int(self.out_params[k_pos][0])) + " " + str(int(self.out_params[k_pos][1])) + " " + k_shield+ " " + k_shield)
        else:
            print(str(int(self.out_params[k_pos][0])) + " " + str(int(self.out_params[k_pos][1])) + " " + str(int(self.out_params[k_thrust])))


class Map:
    def __init__(self, laps=0, num_of_checks=0):
        self.checks = []
        self.laps = laps
        self.num_of_checks = num_of_checks

    def add_check(self, i_check: Check):
        self.checks.append(i_check)

    def get_next_check(self, pos, dif_pos=1):
        next_pos = (pos + dif_pos)%self.num_of_checks
        return {k_pos:next_pos,k_check:copy(self.checks[next_pos])}

    def get_next_check_p(self, pos, dif_pos=1):
        return self.get_next_check(pos, dif_pos)[k_check].p

    def get_check(self, pos):
        return self.get_next_check(pos, 0)

    def get_check_p(self, pos):
        return self.get_check(pos)[k_check].p


def iterate_until_dist(dist, v0):
    max = 100
    a = np.zeros(max)
    v = np.zeros(max)
    e = np.zeros(max)
    a[0]=ai(v0)
    v[0]=v0
    t = 0
    while e[t] < dist and t+1 < max:
        a[t] = ai(v[t])
        v[t+1] = vii(v[t], a[t])
        e[t+1] = eii(e[t], v[t])
        t += 1
    return {"t": t, 'a':a[t], 'v':v[t], 'e':e[t]}


class Game:
    def __init__(self):
        self.map = Map()
        self.enemy_pod_1 = Pod('e1')
        self.enemy_pod_2 = Pod('e2')
        self.player_pod_1 = Pod('p1')
        self.player_pod_2 = Pod('p2')
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
            check_x , check_y = [int(i) for i in input().split()]
            self.map.add_check(Check(check_x, check_y))

    def receive_input(self):
        self.player_pod_1.update_status()
        self.player_pod_2.update_status()
        self.enemy_pod_1.update_status()
        self.enemy_pod_2.update_status()

    def send_output(self):
        self.player_pod_1.send_output()
        self.player_pod_2.send_output()
        self.step +=1

    def calculate_point_speed(self, pod:Pod):

        next_current_angle = v_ang(self.map.get_next_check_p(pod.check_id) - self.map.get_check_p(pod.check_id))
        speed_on_point = v_max - (v_max - v_min)/math.pi * abs(next_current_angle - pod.a)
        if speed_on_point < v_min:
            speed_on_point = v_min

        # print(str(speed_on_point) + ' = '
        #       + str(v_max) + ' - ( ' + str(v_max) + ' - ' + str(v_min) + ')/math.pi * ' + str(
        #     abs(next_current_angle - pod.a)), file=sys.stderr)

        #print("speed_on_point: " + str(speed_on_point), file=sys.stderr)
        return speed_on_point

    def calculate_thrusts(self):
        self.calculate_thrust(self.player_pod_1)
        self.calculate_thrust(self.player_pod_2)

    def calculate_dests(self):
        self.calculate_dest(self.player_pod_1)
        self.calculate_dest(self.player_pod_2)

    def calculate_thrust(self, pod:Pod):
        # print("Calculating thrust", file=sys.stderr)
        self.slow_down = self.should_i_slowdown(self.calculate_point_speed(pod), pod)
        #print("Angle diff: " + str(self.player_pod.a_to_check * 180 / math.pi), file=sys.stderr)
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
                and abs(pod.a - v_ang(pod.v))* deg < 10 \
                and norm(self.map.get_check_p(pod.check_id)- pod.p) > 5000:
            pod.out_params[k_type] = k_boost


    def get_speed_check_ang(self, to_point, pod:Pod):
        # print("check: " + str(self.map.get_current_check().p) +
        #       "- point: " + str(pod.p) +
        #       " speed: " + str(pod.v), file=sys.stderr)
        return ang(to_point - pod.p, pod.v)


    def calc_ang(self, pod:Pod):
        aNC = ang_set(v_ang(self.map.get_check_p(pod.check_id+1) - self.map.get_check_p(pod.check_id)))
        aP = ang_set(pod.a)
        if aNC > aP:
            med = ang_set((aNC - aP)/2)
            beta = aP - med
        else:
            med = ang_set((aP - aNC)/2)
            beta = aNC - med

        print ('aNC:' + str(aNC*deg) + ' aP: ' + str(aP*deg) + ' med: ' + str(med*deg) + ' beta = ' + str(beta*deg), file=sys.stderr)
        return ang_set(beta)


    def correct_dest(self, pod:Pod):
        pointing_to = copy(self.map.get_check_p(pod.check_id))
        beta = self.calc_ang(pod)
        print("beta: " + str(beta), file=sys.stderr)
        pointing_to[0]+=600*math.cos(beta)
        pointing_to[1]+=600*math.sin(beta)
        return pointing_to

    def calculate_dest(self, pod:Pod):
        # print("angle to check: " + str(self.get_speed_check_ang() * 180 / math.pi), file=sys.stderr)
        pointing_to = self.correct_dest(pod)
        print(str(pod.name) + ' check_id: ' + str(pod.check_id) + '->'+ str(self.map.get_check_p(pod.check_id)), file=sys.stderr)
        # pointing_to = self.map.get_check_p(pod.check_id)
        if abs(self.get_speed_check_ang(pointing_to, pod) * deg) < 85:
            pod.out_params[k_pos] = rotate(pod.p, pointing_to, - self.get_speed_check_ang(pointing_to, pod))
        else:
            pod.out_params[k_pos] = pointing_to
        if self.slow_down:
            pod.out_params[k_pos] = self.map.get_check_p(pod.check_id + 1)

    def should_i_slowdown(self, v_d, pod:Pod):
        dist = np.linalg.norm(pod.p - self.map.get_check_p(pod.check_id))
        v_proj = np.dot(pod.v, u_vec(self.map.get_check_p(pod.check_id)-pod.p))
        #print("dist: " + str(dist), file=sys.stderr)
        #print("v_proj: " + str(v_proj), file=sys.stderr)
        #t = t_e(dist - 600, v_proj)
        info = iterate_until_dist(dist - 600, v_proj)
        # info = {'t':0, 'a':0, 'e':0, 'v':0, }
        #print("t_point: " + str(info['t']), file=sys.stderr)
        #print("a_point: " + str(info['a']), file=sys.stderr)
        #print("e_point: " + str(info['e']), file=sys.stderr)
        #print("v_point: " + str(info['v']), file=sys.stderr)
        if pod.turning_started:
            return True
        elif info['v'] > v_d:
            pod.turning_started = True
            return True
        return False


game = Game()
game.initial_input()
# game loop
while True:
    # next_checkpoint_x: x position of the next check point
    # next_checkpoint_y: y position of the next check point
    # next_checkpoint_dist: distance to the next checkpoint
    # next_checkpoint_angle: angle between your pod orientation and the direction of the next checkpoint
    game.receive_input()

    # Write an action using print
    # To debug: print("Debug messages...", file=sys.stderr)

    # You have to output the target position
    # followed by the power (0 <= thrust <= 100) or "BOOST"
    # i.e.: "x y thrust"
    game.calculate_thrusts()
    game.calculate_dests()
    game.send_output()