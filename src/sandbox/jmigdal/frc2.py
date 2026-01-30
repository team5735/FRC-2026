from typing import List, Union, Dict
from typing import Tuple
from dataclasses import dataclass

import os, sys, datetime, time
import pprint
import json
import math
import numpy as np
import cv2

from networktables import NetworkTables

from wpimath.geometry import *
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField, AprilTag

def group_by(lst, key=lambda x:x):
    out = {}
    for x in lst:
        k = key(x)
        if k not in out:
            out[k] = []
        out[k].append(x)
    return out

def list2dict(lst, key=lambda x:x):
    return {key(x): x for x in lst}

@dataclass
class Pose:
    x: float # meters
    y: float # meters
    heading: float # degrees

def ir(x): return int(round(x))
def in2m(x): return x*2.54/100.0
def m2in(x): return x*100/2.54
def d2r(x): return x*math.pi/180.0
def r2d(x): return x*180.0/math.pi

def p322d(p: Pose3d): return Pose2d(p.X(), p.Y(), p.rotation().z)


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

def draw_oriented_rectangle(img,
                            pose: Pose2d,
                            x_len: float,
                            y_len: float,
                            ppm: float = 100.0,
                            color = [255,255,255],
                            front_color = [0, 255, 0]):
    """ draws a rectangle on an image, using field coordinates and meters
    x_len, y_len: lengths, in meters, of the rectangle's extent along it's own x- and y- axes
                  in its own frame, +x points ahead of it, +y goes left
                  rectangle origin is middle of rectangle
    ppm: pixels per meter
    color: BGR color for the rectangle outline
    front_color: BGR color for the front edge of the rectangle
    pose: Pose2d, with x,y in meters, heading in radians
          coordinates/heading are in wpiblue frame -- alliance right wall
          wpiblue origin will be drawn in image upper-right corner,
          x goes left, y goes down
          heading of 0 radians faces left in the image (goes downfield from blue alliance on right)
    """
    img_h, img_w = img.shape[:2]

    # put rectangle center in image
    cx = int(img_w - pose.x * ppm)
    cy = int(pose.y * ppm)

    # 0 == facing left in image
    yaw = pose.rotation().radians()

    half_x = x_len / 2.0
    half_y = y_len / 2.0

    # Corners in rectangle frame (meters)
    corners = [
        (+half_x, +half_y),  # front-left
        (+half_x, -half_y),  # front-right
        (-half_x, -half_y),  # back-right
        (-half_x, +half_y),  # back-left
    ]

    pts = []
    for fx, fy in corners:
        # Rotate in field frame
        rx = fx * math.cos(yaw) - fy * math.sin(yaw)
        ry = fx * math.sin(yaw) + fy * math.cos(yaw)

        px = int(cx - rx * ppm)
        py = int(cy + ry * ppm)

        pts.append((px, py))

    # Outline
    pts_np = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(img, [pts_np], True, (255, 255, 255), 1)

    # Front edge shows yaw
    cv2.line(img, pts[0], pts[1], (0, 255, 0), 2)

    return pts


def draw_bot(image, pose2d, ppm, x_len, y_len):
    draw_oriented_rectangle(image, pose2d, x_len, y_len, ppm, (255, 255, 255), (0, 255, 0))

def draw_tag(
    image,
    tag_id: int,
    pose: Pose2d,
    pixels_per_meter: float = 100.0,
    length_m: float = 0.1651, # 6.5 inches
    width_m: float = 0.08,
):
    """
    Draw an april tag on the field
    see draw_oriented_rectangle for details
    """
    img_h, img_w = image.shape[:2]

    pts = draw_oriented_rectangle(image, pose, width_m, length_m, pixels_per_meter, (255, 255, 255), (0, 255, 0))

    # ID label
    mnx,mxx = min([x[0] for x in pts]), max([x[0] for x in pts])
    mny,mxy = min([x[1] for x in pts]), max([x[1] for x in pts])
    px = mxx+4
    py = (mxy+mny)//2
    cv2.putText(
        image,
        f"{tag_id}",
        (px, py),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )


# ---- connection callback (optional but useful) ----

def connection_listener(connected, info):
    print(f"{'Connected to' if connected else 'Disconnected from'} {info.remote_id}")


def new_april_tag(id, p3d):
    t = AprilTag()
    t.ID = id
    t.pose = p3d
    return t

def ntvis1():
    if 0:
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

        # get robot pose from network tables
        robot_pose = Pose(*table.getNumberArray("Field/Robot", [0,0,0]))
        print('robot:',robot_pose)

    # stub robot pose
    if 1:
        robot_pose = Pose2d(15.0, 1.0, d2r(135)) # upper-left, pointing to lower-right
        print('stub robot pose',robot_pose)

    if 1: # stub april tags
        tags = [
            new_april_tag(1, Pose3d(Translation3d(1.0, 1.0, 0), Rotation3d(0,0,0))),
            new_april_tag(2, Pose3d(Translation3d(2.0, 1.0, 0), Rotation3d(0,0,0))),
            new_april_tag(3, Pose3d(Translation3d(1.0, 2.0, 0), Rotation3d(0,0,0))),
            new_april_tag(4, Pose3d(Translation3d(2.0, 2.0, 0), Rotation3d(0,0,0))),
        ]

    if 0: # get 2026 tags from json file
        print('getting tags from 2026 file')
        layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)
        print('field: %sx%s meters'%(layout.getFieldLength(), layout.getFieldWidth()))
        print('field: %d tags'%(len(layout.getTags())))
        tags = layout.getTags()
        id2tag = list2dict(tags, key=lambda t:t.ID)
        print('tag 10:',id2tag[10].pose)
        # for tag in layout.getTags():
        #     print(tag.ID, tag.pose)
        #     print('  ',p322d(tag.pose))

    if 0: # get tags from network tables
        tags  = table.getNumberArray("Field/pose_score_left", [])
        tags += table.getNumberArray("Field/pose_score_right", [])

        assert (len(tags) % 3) == 0

        tag_poses = [Pose2d(tags[i], tags[i+1], tags[i+2]) for i in range(0, len(tags), 3)]
        tags = [AprilTag(ix, pose) for ix, pose in enumerate(tag_poses)]
        id2tag = list2dict(tags, key=lambda t:t.ID)
        print(tags)
        print('tag 10:',id2tag[10].pose)

    # print(tag_poses[0])
    # pprint.pprint(tag_poses)
    # tag_poses.sort(key=lambda t: dist((t.x,t.y),(robot_pose.x, robot_pose.y)))

    tags.sort(key=lambda t: dist((t.pose.X(),t.pose.Y()),(robot_pose.X(), robot_pose.Y())))


    a = (robot_pose.x, robot_pose.y)
    b = (tags[0].x, tags[0].y)
    c = (tags[1].x, tags[1].y)
    d = (tags[2].x, tags[2].y)

    a = (ir(100*a[0]), ir(100*a[1]))
    b = (ir(100*b[0]), ir(100*b[1]))
    c = (ir(100*c[0]), ir(100*c[1]))
    d = (ir(100*d[0]), ir(100*d[1]))

    print('robot',a)
    print('b',b)
    print('c',c)
    print('d',d)



    # ---- image setup ----

    width, height = 1600, 800
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

    for tagid in id2tag:
        draw_tag(img, tagid, p322d(id2tag[tagid].pose))

    draw_bot(img, Pose2d(15, 1, Rotation2d(d2r(180-10))), 100, 1, 1)

    # ---- display ----

    cv2.imshow("FRC Geometry Debug View", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()





def main():
    return ntvis1()

if __name__ == '__main__':
    main()
