#!/usr/bin/python

import unittest

from sickday2016 import is_path_blocked, is_in_loading_zone


class Sickday2016Test(unittest.TestCase):

    def test_is_path_blocked(self):
        remission = [100]*274
        data = [1000]*271
        self.assertFalse(is_path_blocked(data, remission))

        data[135] = 150
        self.assertTrue(is_path_blocked(data, remission))

        data[135] = 0  # timeout
        self.assertFalse(is_path_blocked(data, remission))

    def test_is_in_loading_zone(self):
        # TODO replace by real coordinates
        self.assertTrue(is_in_loading_zone((0,0,0)))


if __name__ == "__main__":
    unittest.main()

#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4  
