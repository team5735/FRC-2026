from typing import List, Union, Dict
from dataclasses import dataclass

from typing import Tuple
import os, sys
import pprint
import json
import math
import numpy as np
import cv2

import time
from networktables import NetworkTables


import cv2
import numpy as np
import math

@dataclass
class Pose:
    x: float # meters
    y: float # meters
    heading: float # degrees

def ir(x): return int(round(x))

def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def draw_point(img, p, label, color=(0, 255, 0)):
    cv2.circle(img, p, 5, color, -1)
    cv2.putText(
        img,
        label,
        (p[0] + 5, p[1] - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        1,
        cv2.LINE_AA
    )

def draw_line_with_distance(img, p1, p2, color=(255, 0, 0)):
    cv2.line(img, p1, p2, color, 2)

    d = dist(p1, p2)
    mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)

    cv2.putText(
        img,
        f"{d:.1f}",
        mid,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        1,
        cv2.LINE_AA
    )



# ---- connection callback (optional but useful) ----

def connection_listener(connected, info):
    print(f"{'Connected to' if connected else 'Disconnected from'} {info.remote_id}")


def ntvis1():
    NetworkTables.addConnectionListener(connection_listener, immediateNotify=True)

    # ---- initialize ----
    # Local simulation
    NetworkTables.initialize(server="127.0.0.1")

    # ---- wait for connection ----
    print("Waiting for NetworkTables connection...")
    while not NetworkTables.isConnected():
        time.sleep(0.1)

    print("Connected to NetworkTables")


    # Table path (match robot code)
    table = NetworkTables.getTable("SmartDashboard")

    robot_pose = Pose(*table.getNumberArray("Field/Robot", [0,0,0]))
    print(robot_pose)

    if 1: # get 2026 tags from json file
        print('getting tags from 2026 file')
        tags = []
        with open('../../resources/FRC2026_ANDYMARK.fmap') as f:
            j = json.load(f)
        for tag in j['fiducials']:
            tid = tag['id']
            xform = np.asarray(tag['transform'])
            xform = xform.reshape((4,4))
            print(xform)
            tags.append(Pose(tag['x'], tag['y'], tag['heading']))
        tag_poses = tags
        return

    if 0: # get 2025 tags from network tables
        tags  = table.getNumberArray("Field/pose_score_left", [])
        tags += table.getNumberArray("Field/pose_score_right", [])

        assert (len(tags) % 3) == 0

        tag_poses = [Pose(tags[i], tags[i+1], tags[i+2]) for i in range(0, len(tags), 3)]

    print(tag_poses[0])

    pprint.pprint(tag_poses)

    tag_poses.sort(key=lambda t: dist((t.x,t.y),(robot_pose.x, robot_pose.y)))


    # stub data
    a = (100, 100)
    b = (300, 120)
    c = (180, 300)
    d = (400, 260)

    a = (robot_pose.x, robot_pose.y)
    b = (tag_poses[0].x, tag_poses[0].y)
    c = (tag_poses[1].x, tag_poses[1].y)
    d = (tag_poses[2].x, tag_poses[2].y)

    a = (ir(100*a[0]), ir(100*a[1]))
    b = (ir(100*b[0]), ir(100*b[1]))
    c = (ir(100*c[0]), ir(100*c[1]))
    d = (ir(100*d[0]), ir(100*d[1]))

    print('robot',a)
    print('b',b)
    print('c',c)
    print('d',d)



    # ---- image setup ----

    width, height = 600, 600
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)  # dark background

    # ---- draw points ----

    draw_point(img, a, "A")
    draw_point(img, b, "B")
    draw_point(img, c, "C")
    draw_point(img, d, "D")

    # ---- draw lines from A ----

    draw_line_with_distance(img, a, b)
    draw_line_with_distance(img, a, c)
    draw_line_with_distance(img, a, d)

    # ---- display ----

    cv2.imshow("FRC Geometry Debug View", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()





def main():
    return ntvis1()

if __name__ == '__main__':
    main()
