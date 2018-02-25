from Dynamics import *
from Constants import *


class Pod:
    def __init__(self, name, i_x=-1, i_y=-1):
        self.name = name
        self.p = np.array([i_x, i_y])
        self.p_prev = np.array([i_x, i_y])
        self.v = np.array([0, 0])
        self.v_ang = vector_ang(self.v)
        self.v_prev = np.array([0, 0])
        self.v_prev_ang = vector_ang(self.v_prev)
        self.a = 0
        self.a_prev = 0
        self.check_id = -1
        self.out_params = {}

        self.turning_started = False
        self.boost_used = False

    def __eq__(self, other):
        return self.name == other

    def __ne__(self, other):
        return self.name != other

    def add_step(self, i_p: np.array, i_v: np.array, i_a: int):
        print("Pod " + str(self.name) + ' i_p: ' + str(i_p) + ' i_v: ' + str(i_v), file=sys.stderr)
        self.p_prev = self.p
        self.p = i_p
        self.v_prev = self.v
        self.v_prev_ang = self.v_ang
        self.v = i_v
        self.v_ang = vector_ang(self.v)
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
        # print('p: ' + str(self.p) + ' p_prev: ' + str(self.p_prev), file=sys.stderr)
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
        self.add_step(np.array([x, y]), np.array([vx, vy]), angle/deg)
        if self.collision():
            self.turning_started = False

    def send_output(self):
        if self.out_params[k_type] == k_boost:
            print("BOOSTING", file=sys.stderr)
            self.boost_used = True
            print(str(int(self.out_params[k_pos][0])) + " "
                  + str(int(self.out_params[k_pos][1]))
                  + " " + k_boost + " " + k_boost)
        elif self.out_params[k_type] == k_shield:
            print("SHIELDING", file=sys.stderr)
            print(str(int(self.out_params[k_pos][0]))
                  + " " + str(int(self.out_params[k_pos][1]))
                  + " " + k_shield + " " + k_shield)
        else:
            print(str(int(self.out_params[k_pos][0]))
                  + " " + str(int(self.out_params[k_pos][1]))
                  + " " + str(int(self.out_params[k_thrust])))
