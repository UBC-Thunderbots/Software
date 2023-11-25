"""
The code in this file is adapted from
https://github.com/linuxlewis/tripy/tree/master

MIT License

Copyright (c) 2017 Sam Bolgert

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import math
import sys
from collections import namedtuple
from typing import List, Tuple

Point = namedtuple("Point", ["x", "y", "index"])


def earclip(polygon: List[Tuple[float, float]]) -> List[Tuple[int, int, int]]:
    """Triangulates a polygon using a simple earclipping algorithm.

    Implementation reference:
    https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
    
    Example usage:
    >>> polygon = [(0, 1), (-1, 0), (0, -1), (1, 0)]
    >>> triangles = tripy.earclip(polygon)
    >>> triangles
    [(3, 0, 1), (3, 1, 2)]
    
    :param polygon: an array of 2-tuples of the cartesian points of the polygon
    :returns: an array of 3-tuples where each item in the tuple is a index into
              the polygon array (for a polygon with n points, (n - 2) triangles 
              will be returned)
    """
    ear_vertex = []
    triangles = []

    polygon = [Point(point[0], point[1], index) for index, point in enumerate(polygon)]

    if __is_clockwise(polygon):
        polygon.reverse()

    point_count = len(polygon)
    for i in range(point_count):
        prev_point = polygon[i - 1]
        point = polygon[i]
        next_point = polygon[(i + 1) % point_count]

        if _is_ear(prev_point, point, next_point, polygon):
            ear_vertex.append(point)

    while ear_vertex and point_count >= 3:
        ear = ear_vertex.pop(0)
        i = polygon.index(ear)
        prev_index = i - 1
        prev_point = polygon[prev_index]
        next_index = (i + 1) % point_count
        next_point = polygon[next_index]

        polygon.remove(ear)
        point_count -= 1
        triangles.append((prev_point.index, ear.index, next_point.index))
        if point_count > 3:
            prev_prev_point = polygon[prev_index - 1]
            next_next_index = (i + 1) % point_count
            next_next_point = polygon[next_next_index]

            groups = [
                (prev_prev_point, prev_point, next_point, polygon),
                (prev_point, next_point, next_next_point, polygon),
            ]
            for group in groups:
                p = group[1]
                if _is_ear(*group):
                    if p not in ear_vertex:
                        ear_vertex.append(p)
                elif p in ear_vertex:
                    ear_vertex.remove(p)

    return triangles


def __is_clockwise(polygon):
    s = 0
    polygon_count = len(polygon)
    for i in range(polygon_count):
        point = polygon[i]
        point2 = polygon[(i + 1) % polygon_count]
        s += (point2.x - point.x) * (point2.y + point.y)
    return s > 0


def __is_convex(prev, point, next):
    return __triangle_sum(prev.x, prev.y, point.x, point.y, next.x, next.y) < 0


def _is_ear(p1, p2, p3, polygon):
    ear = (
        __contains_no_points(p1, p2, p3, polygon)
        and __is_convex(p1, p2, p3)
        and __triangle_area(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y) > 0
    )
    return ear


def __contains_no_points(p1, p2, p3, polygon):
    for pn in polygon:
        if pn in (p1, p2, p3):
            continue
        elif __is_point_inside(pn, p1, p2, p3):
            return False
    return True


def __is_point_inside(p, a, b, c):
    area = __triangle_area(a.x, a.y, b.x, b.y, c.x, c.y)
    area1 = __triangle_area(p.x, p.y, b.x, b.y, c.x, c.y)
    area2 = __triangle_area(p.x, p.y, a.x, a.y, c.x, c.y)
    area3 = __triangle_area(p.x, p.y, a.x, a.y, b.x, b.y)
    areadiff = abs(area - sum([area1, area2, area3])) < math.sqrt(
        sys.float_info.epsilon
    )
    return areadiff


def __triangle_area(x1, y1, x2, y2, x3, y3):
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)


def __triangle_sum(x1, y1, x2, y2, x3, y3):
    return x1 * (y3 - y2) + x2 * (y1 - y3) + x3 * (y2 - y1)
