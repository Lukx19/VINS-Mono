import os
import argparse
import math

header = "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []\n"

def toQuaternion(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # w,x,y,z
    q = [0,0,0,0]
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr
    return q

def toString(float_list):
    return ','.join([str(e) for e in float_list])+"\n"

if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory.
    ''')

    parser.add_argument('source', help='original ground truth (format: timestamp tx ty theta)')
    parser.add_argument('target', help='ground truth in EuroC format (format: timestamp tx ty tz qw qx qy qz)')
    args = parser.parse_args()

    with open(args.source,"r") as read_f:
        with open(args.target, "w") as write_f:
            data = read_f.read()
            lines = data.replace(","," ").replace("\t"," ").split("\n")
            striped = [[float(v.strip()) for v in line.split(" ") if v.strip() != ""] for line in lines if len(line) > 0 and line[0] != "#"]

            out_lines = [toString([line[0], line[1], 0, *toQuaternion(line[2], 0, 0)]) for line in striped]
            # print(out_lines)
            write_f.write(header)
            write_f.writelines(out_lines)

