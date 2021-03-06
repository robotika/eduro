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
        self.assertFalse(is_in_loading_zone((0, 1.0, 0), (None, None)))


    def test_is_path_blocked(self):

        # sparse reflection
        # m:\git\eduro\logs\2016-10-03\czu5\src_laser_161003_173236.log
        data = [64, 70, 74, 67, 64, 65, 66, 58, 65, 60, 58, 57, 62, 57, 65, 58, 64, 63, 69,
        62, 67, 67, 67, 60, 58, 67, 63, 58, 62, 69, 64, 61, 61, 66, 65, 58, 70,
        63, 63, 57, 56, 66, 61, 60, 56, 58, 57, 59, 62, 61, 59, 57, 59, 62, 64,
        64, 68, 65, 66, 65, 65, 42, 33, 24, 17, 17, 15, 15, 15, 15, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 19, 26, 33, 42, 57, 61, 1670,
        1673, 1688, 1697, 1702, 1717, 1726, 1732, 1750, 1759, 1779, 1796, 1808,
        1823, 1844, 1857, 1879, 1893, 1914, 1940, 1958, 1988, 2017, 2041, 2074,
        2096, 2125, 2165, 2191, 2225, 2271, 2305, 2344, 2342, 2307, 2271, 2265,
        2310, 2294, 2261, 2232, 2209, 2186, 2160, 2135, 2114, 2099, 2074, 2055,
        2042, 2021, 2008, 1988, 1980, 1963, 1952, 1938, 1926, 1915, 1849, 1812,
        1800, 1798, 1786, 1779, 1778, 1767, 1769, 1755, 1753, 1760, 1755, 1752,
        1749, 1750, 1749, 1749, 1745, 1750, 1753, 1756, 1754, 1763, 1759, 1765,
        1776, 1783, 1786, 1790, 1797, 1807, 1811, 1826, 1833, 1851, 1850, 1853,
        1859, 1882, 1887, 1903, 1926, 1939, 1956, 1974, 1989, 2012, 2028, 2048,
        2071, 2097, 2122, 2145, 2175, 2211, 2237, 2268, 2299, 2337, 2379, 2416,
        2387, 2342, 2300, 2281, 2321, 2362, 2393, 2369, 2338, 2306, 2281, 2252,
        2226, 2202, 2177, 2156, 2140, 2109, 2094, 2073, 2067, 2044, 2021, 2015,
        1994, 1993, 1970, 1957, 1953, 1941, 1930, 1919, 1909, 1901, 1899, 1888,
        1885, 1873, 1859, 1844, 1833, 1831, 1818, 1822, 1826, 1815, 1826, 1831,
        1822, 1826, 1830, 1846, 1854, 1868, 1871, 1876, 1885, 1893, 1893] 
        self.assertFalse(is_path_blocked(data))

        # hitting the boundary
        # m:\git\eduro\logs\2016-10-03\czu3\src_laser_161003_171016.log
        data = [240, 244, 242, 251, 255, 261, 266, 275, 280, 286, 288, 288, 292, 280, 266,
        267, 264, 264, 266, 265, 256, 256, 259, 257, 259, 262, 258, 256, 251,
        213, 202, 1789, 1780, 1773, 1759, 1756, 1744, 1737, 1730, 1716, 1714,
        1701, 1704, 1692, 1694, 1685, 1684, 1689, 3211, 1221, 1221, 1217, 1219,
        1215, 1211, 1135, 957, 556, 393, 332, 288, 257, 235, 216, 202, 193,
        175, 171, 160, 150, 137, 139, 129, 126, 117, 115, 110, 101, 100, 95,
        94, 91, 89, 81, 81, 74, 77, 81, 75, 75, 71, 74, 72, 74, 75, 73, 72, 67,
        65, 60, 64, 64, 55, 48, 56, 55, 48, 43, 50, 45, 50, 47, 46, 53, 48, 45,
        45, 44, 46, 51, 49, 55, 49, 42, 55, 53, 48, 44, 51, 46, 48, 48, 50, 52,
        49, 53, 45, 46, 51, 47, 52, 52, 49, 56, 54, 48, 59, 50, 45, 49, 51, 52,
        52, 52, 50, 50, 49, 53, 50, 55, 56, 50, 51, 55, 51, 45, 45, 56, 46, 54,
        50, 48, 47, 44, 36, 45, 47, 50, 50, 47, 49, 46, 48, 47, 54, 53, 61, 59,
        62, 66, 73, 74, 77, 72, 68, 74, 69, 71, 78, 72, 77, 76, 75, 72, 83, 82,
        84, 81, 84, 86, 92, 97, 98, 104, 108, 112, 119, 126, 133, 138, 150,
        160, 173, 178, 192, 200, 216, 236, 280, 368, 481, 626, 0, 2034, 2035,
        2034, 1557, 1458, 1456, 1425, 1417, 1397, 1422, 1467, 1525, 1570, 1640,
        1840, 1841, 1854, 1856, 1868, 1874, 1891, 1898, 1918, 1932, 1948, 1955,
        1983, 1992, 2017, 2033, 2052, 2077, 2095, 2124, 2141, 2174, 2196,
        2225]
        self.assertTrue(is_path_blocked(data))



if __name__ == "__main__":
    unittest.main()

#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4  
