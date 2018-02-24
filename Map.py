from Constants import *

from Check import Check
from copy import copy


class Map:
    def __init__(self, laps=0, num_of_checks=0):
        self.checks = []
        self.laps = laps
        self.num_of_checks = num_of_checks

    def add_check(self, i_check: Check):
        self.checks.append(i_check)

    def get_next_check(self, pos, dif_pos=1):
        next_pos = (pos + dif_pos) % self.num_of_checks
        return {k_pos: next_pos, k_check: copy(self.checks[next_pos])}

    def get_next_check_p(self, pos, dif_pos=1):
        return self.get_next_check(pos, dif_pos)[k_check].p

    def get_check(self, pos):
        return self.get_next_check(pos, 0)

    def get_check_p(self, pos):
        return self.get_check(pos)[k_check].p
