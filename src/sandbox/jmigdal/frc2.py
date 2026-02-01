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


class TagHighlightAnimation(object):
    def __init__(self, id, pose, t_secs:float, ppm, pad_x, pad_y):
        self.id = id
        self.pose = pose


        self.cur_ts = datetime.datetime.now()
        self.dur = datetime.timedelta(seconds=t_secs)
        self.start_ts = self.cur_ts
        self.end_ts = self.start_ts + self.dur

        self.highlight_color = (60,20,220) # crimson red in bgr
        self.normal_color = (255,255,255)
        self.normal_front_color = (0,255,0)
        self.normal_text_color = (0, 255, 255)

        self.ppm = ppm
        self.pad_x = pad_x
        self.pad_y = pad_y

    def draw(self, img):
        self.cur_ts = datetime.datetime.now()
        elapsed = self.cur_ts - self.start_ts

        # meld highlight and normal colors according to % done
        alpha = elapsed / self.dur
        if alpha > 1.0:
            alpha = 1.0
        draw_color = [0,0,0]
        front_color = [0,0,0]
        text_color = [0,0,0]
        # blend colors
        for c in range(3):
            draw_color[c] = ir(alpha*self.normal_color[c] + (1-alpha)*self.highlight_color[c])
            front_color[c] = ir(alpha*self.normal_front_color[c] + (1-alpha)*self.highlight_color[c])
            text_color[c] = ir(alpha*self.normal_text_color[c] + (1-alpha)*self.highlight_color[c])
            text_color[c] = self.highlight_color[c]

        draw_tag(img, self.id, p322d(self.pose), self.ppm, pad_x=self.pad_x, pad_y=self.pad_y,
                 color=tuple(draw_color),
                 front_color=tuple(front_color),
                 text_color=tuple(text_color))


    def is_expired(self):
        return self.cur_ts > self.end_ts

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
    if label:
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

def draw_line_with_distance(img, p1, p2, text, color=(255, 0, 0)):
    cv2.line(img, p1, p2, color, 2)

    mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)

    cv2.putText(
        img,
        text,
        mid,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        [255,255,255],
        6,
        cv2.LINE_AA
    )

    cv2.putText(
        img,
        text,
        mid,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        1,
        cv2.LINE_AA
    )

def field_point_to_pixel(x_meters, y_meters, imgw, imgh, ppm, pad_x=0, pad_y=0):
    """ converts physical dimension field coords to pixel location in img
        x_meters: float field x extent in meters. origin is blue alliance right corner
        y_meters: float field y extent in meters
        imgw: int image width in pixels
        imgh: int image height in pixels
        ppm: int/float pixels per meter
        pad_x, pad_y: int pixels to pad the field by on each side (to prevent clipping)
                      conceptually shifts the image origin by (-pad_x, -pad_y) pixels
    """
    if 1: # blue alliance is upper right when looking at the field top down
        px = int(-pad_x + imgw - x_meters * ppm)
        py = int(pad_y + y_meters * ppm)

    if 0: # blue alliance is lower left when looking at the field top down
        px = int(pad_x + x_meters * ppm)
        py = int(-pad_y + imgh - y_meters * ppm)

    return (px, py)

def draw_arc(img,
             pose: Pose2d,
             radius_m: float,
             arc_span_deg: float,
             ppm: float = 100.0,
             color = [255,255,255],
             pad_x=0,
             pad_y=0
             ):
    img_h, img_w = img.shape[:2]
    cx,cy = field_point_to_pixel(pose.X(), pose.Y(), img_w, img_h, ppm, pad_x, pad_y)
    r_px = int(radius_m * ppm)

    # Heading
    heading_rad = pose.rotation().radians()
    heading_deg = r2d(heading_rad)

    # Arc angles in WPILib frame
    start_deg = heading_deg - arc_span_deg / 2
    end_deg   = heading_deg + arc_span_deg / 2

    # Convert to OpenCV frame - 0=right, clockwise positive
    if 1: # we're 0=left, counterclockwise positive
        start_cv = 180-start_deg
        end_cv   = 180-end_deg

    if 0: # we're 0=right, counterclockwise positive
        start_cv = -start_deg
        end_cv   = -end_deg


    # OpenCV expects start < end in clockwise space
    if end_cv < start_cv:
        start_cv, end_cv = end_cv, start_cv

    cv2.ellipse(
        img,
        center=(cx,cy),
        axes=(r_px, r_px),
        angle=0.0,            # no ellipse rotation; circle
        startAngle=start_cv,
        endAngle=end_cv,
        color=color,
        thickness=1,
        lineType=cv2.LINE_AA,
    )
def draw_oriented_rectangle(img,
                            pose: Pose2d,
                            x_len: float,
                            y_len: float,
                            ppm: float = 100.0,
                            color = (255,255,255),
                            front_color = (0, 255, 0),
                            pad_x=0,
                            pad_y=0):
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
    cx,cy = field_point_to_pixel(pose.X(), pose.Y(), img_w, img_h, ppm, pad_x, pad_y)

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

        px,py = field_point_to_pixel(pose.X()+rx, pose.Y()+ry, img_w, img_h, ppm, pad_x, pad_y)
        pts.append((px, py))

    # Outline
    pts_np = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(img, [pts_np], True, color, 1)

    # Front edge shows yaw
    cv2.line(img, pts[0], pts[1], front_color, 2)

    return pts


def draw_bot(image, pose2d, ppm, x_len, y_len, pad_x=0, pad_y=0):
    return draw_oriented_rectangle(image, pose2d, x_len, y_len, ppm, (255, 255, 255), (0, 255, 0), pad_x, pad_y)

def draw_tag(
    image,
    tag_id: int,
    pose: Pose2d,
    ppm: float = 100.0,
    length_m: float = in2m(6.5),
    width_m: float = in2m(2),
    pad_x=0,
    pad_y=0,
    color=(255,255,255),
    front_color=(0, 255, 0),
    text_color=(0, 255, 255)
):
    """
    Draw an april tag on the field
    see draw_oriented_rectangle for details
    """
    img_h, img_w = image.shape[:2]

    pts = draw_oriented_rectangle(image, pose, width_m, length_m, ppm, color, front_color, pad_x, pad_y)

    # ID label
    if tag_id is not None:
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
            text_color,
            1,
            cv2.LINE_AA,
        )
    return pts


# ---- connection callback (optional but useful) ----

def connection_listener(connected, info):
    print(f"{'Connected to' if connected else 'Disconnected from'} {info.remote_id}")

# give AprilTag one line constructor
def new_april_tag(id, p3d):
    t = AprilTag()
    t.ID = id
    t.pose = p3d
    return t

def get_robot_pose(nt_smart_dashboard_table):
    # stub robot pose
    if nt_smart_dashboard_table is None:
        robot_pose = Pose2d(in2m(12*6), in2m(12*9), d2r(30))
        return robot_pose

    # get robot pose from network tables
    if 'limelight' in nt_smart_dashboard_table.getPath():
        nums = nt_smart_dashboard_table.getNumberArray('botpose_wpiblue', [])
        if not nums:
            return Pose2d(0, 0, 0)
        x,y,yaw = nums[0],nums[1],nums[5]
        yaw = d2r(yaw)
        robot_pose = Pose2d(x,y,yaw)
        return robot_pose
    else:
        x,y,yaw = nt_smart_dashboard_table.getNumberArray("Field/Robot", [0,0,0])
        yaw = d2r(yaw)
        robot_pose = Pose2d(x,y,yaw)
        return robot_pose

def get_detected_tags(ll_table, last_rfs=None):
    if ll_table is None:
        return [10,]

    ids = []
    rfs = ll_table.getNumberArray('rawfiducials',[])

    # check if the last received fiducials haven't changed
    # if they haven't, we haven't detected fiducials this call
    if last_rfs is not None and rfs == last_rfs:
        return [], None

    for ix in range(0,len(rfs),7):
        ids.append(int(rfs[ix]))

    return ids,rfs


def ntvis1():
    robot_len_x = in2m(30) # meters, front-back length
    robot_len_y = in2m(30) # meters

    field_len_x = in2m(650.12)
    field_len_y = in2m(316.64)

    window_w = 1200
    window_h = 800
    pad_x, pad_y = 20,20 # pixel padding on the edges of img to allow for slightly negative drawing indices

    ppm = min(window_w/field_len_x,
              window_h/field_len_y)
    print('ppm:',ppm)

    # now make the image the correct size
    window_w += pad_x
    window_h += pad_y


    sd_table = None
    lll_table = None
    if 1:
        NetworkTables.addConnectionListener(connection_listener, immediateNotify=True)

        # NetworkTables.initialize(server="127.0.0.1")
        NetworkTables.initialize(server="10.57.35.2") # moriarity roborio

        # ---- wait for connection ----
        print("Waiting for NetworkTables connection...")
        while not NetworkTables.isConnected():
            time.sleep(0.1)

        print("Connected to NetworkTables")


        # Table path (match robot code)
        sd_table = NetworkTables.getTable("SmartDashboard")
        lll_table = NetworkTables.getTable("limelight-left")

    if 0: # stub april tags
        tags = [
            new_april_tag(1, Pose3d(Translation3d(1.0, 1.0, 0), Rotation3d(0,0,0))),
            new_april_tag(2, Pose3d(Translation3d(2.0, 1.0, 0), Rotation3d(0,0,0))),
            new_april_tag(3, Pose3d(Translation3d(1.0, 2.0, 0), Rotation3d(0,0,0))),
            new_april_tag(4, Pose3d(Translation3d(2.0, 2.0, 0), Rotation3d(0,0,0))),
        ]
        id2tag = list2dict(tags, key=lambda t:t.ID)

    if 1: # get 2026 tags from json file
        print('getting tags from 2026 file')
        layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)
        print('field: %sx%s meters'%(layout.getFieldLength(), layout.getFieldWidth()))
        print('field: %d tags'%(len(layout.getTags())))
        tags = layout.getTags()
        id2tag = list2dict(tags, key=lambda t:t.ID)
        print('tag 10:',id2tag[10].pose)
        for tag in layout.getTags():
            print(tag.ID, tag.pose)
            print('  ',p322d(tag.pose))

    # create viz
    img = np.zeros((window_h, window_w, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)  # dark background

    animations = []
    _A = TagHighlightAnimation(10, id2tag[10].pose, t_secs=1.0, ppm=ppm, pad_x=pad_x, pad_y=pad_y)
    animations.append(_A)

    detected_tags = []
    last_detected_fiducials = []
    last_detected_fiducials_ts = datetime.datetime.now()
    while True:
        # robot_pose = get_robot_pose(sd_table)
        robot_pose = get_robot_pose(lll_table)
        # robot_pose = Pose2d(robot_pose.translation(), Rotation2d(d2r(-170)))

        # check if we detected new tags
        _detected_tags, _lastrfs = get_detected_tags(lll_table, last_detected_fiducials)

        # we actually detected a new set of tags
        if _lastrfs is not None:
            last_detected_fiducials = _lastrfs
            last_detected_fiducials_ts = datetime.datetime.now()
            detected_tags = _detected_tags

            # add tag highlight indicating which were detected in pose estimation
            for tag_id in detected_tags:
                if tag_id in id2tag:
                    _A = TagHighlightAnimation(tag_id, id2tag[tag_id].pose, t_secs=1.0, ppm=ppm, pad_x=pad_x, pad_y=pad_y)
                    animations.append(_A)

        # find nearest tags to front of robot
        a = [robot_pose.X(), robot_pose.Y()]
        # center of front of bot
        a[0] += math.cos(robot_pose.rotation().radians())*robot_len_x/2
        a[1] += math.sin(robot_pose.rotation().radians())*robot_len_x/2
        robot_front_pose = Pose2d(a[0], a[1], robot_pose.rotation())

        sorted_tags = sorted(tags, key=lambda t: dist((t.pose.X(),t.pose.Y()),a))



        # render scene
        img[:] = (30, 30, 30)  # dark background

        # put april tags on image
        for tagid in id2tag:
            draw_tag(img, tagid, p322d(id2tag[tagid].pose), ppm, pad_x=pad_x, pad_y=pad_y)

        # put robot on image
        draw_bot(img, robot_pose, ppm, robot_len_x, robot_len_y, pad_x, pad_y)

        # pixel location of the center front of bot in the image (used for cf measuring distances in real life)
        a_px = field_point_to_pixel(robot_front_pose.X(), robot_front_pose.Y(), window_w, window_h, ppm, pad_x, pad_y)

        # draw lines and distances to 3 nearest tags
        if 0:
            draw_point(img, a_px, "")
            for tag in sorted_tags[:3]:
                b_px = field_point_to_pixel(tag.pose.X(), tag.pose.Y(), window_w, window_h, ppm, pad_x, pad_y)
                draw_point(img, b_px, "")
                _dist = m2in(dist(a, (tag.pose.X(), tag.pose.Y())))
                draw_line_with_distance(img, a_px, b_px, '%.1f"'%_dist)

        # draw lines and distances to detected tags
        if 1:
            # check if these detections are recent enough, otherwise we aren't seeing any tags right now
            if datetime.datetime.now()-last_detected_fiducials_ts < datetime.timedelta(seconds=0.25):
                for tid in detected_tags:
                    tag = id2tag[tid]
                    b_px = field_point_to_pixel(tag.pose.X(), tag.pose.Y(), window_w, window_h, ppm, pad_x, pad_y)
                    # don't draw the point because of animations below (looks bad)
                    # draw_point(img, b_px, "")
                    _dist = m2in(dist(a, (tag.pose.X(), tag.pose.Y())))
                    draw_line_with_distance(img, a_px, b_px, '%.1f"'%_dist)


        # hub_center_shooting_arc = Pose2d(in2m(181.56), in2m(158.32), d2r(180))
        hub_center_shooting_arc = Pose2d(11.91, 4.02, d2r(0))
        span = 90
        draw_arc(img, hub_center_shooting_arc, in2m(10*12), span, ppm, pad_x=pad_x, pad_y=pad_y)

        # draw animations
        for A in animations:
            A.draw(img)
        animations = [x for x in animations if not x.is_expired()]

        # show image
        cv2.imshow("FRC Geometry Debug View", img)
        key = cv2.waitKey(16)

        if key & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()





def main():
    return ntvis1()

if __name__ == '__main__':
    main()
