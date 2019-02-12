#!/usr/bin/python

# Modified by Raul Mur-Artal
# Automatically compute the optimal scale factor for monocular VO/SLAM.

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements:
# sudo apt-get install python-argparse

"""
This script computes the absolute trajectory error from the ground truth
trajectory and the estimated trajectory.
"""

import sys
import numpy as np
import itertools
import argparse
import math
import os
import json

import evaluate_rpe

import transformations as transf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from matplotlib.patches import Ellipse
from matplotlib import rc
# rc('font',**{'family':'serif','serif':['Cardo']})
# rc('text', usetex=True)


_EPS = np.finfo(float).eps * 4.0

def read_file_list_raw(filename):
    """
    Reads a trajectory from a text file.

    File format:
    The file format is "d1 d2 d3 ...", "d1 d2 d3.." is arbitary data

    Input:
    filename -- File name

    Output:
    dict -- list of data tuples
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n")
    sriped = [[float(v.strip()) for v in line.split(" ") if v.strip() != ""] for line in lines if len(line) > 0 and line[0] != "#"]
    return sriped

def read_file_list(filename):
    """
    Reads a trajectory from a text file.

    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.

    Input:
    filename -- File name

    Output:
    dict -- dictionary of (stamp,data) tuples

    """
    sriped = read_file_list_raw(filename)
    dt = [(float(l[0]), list(itertools.chain.from_iterable([l[1:4],l[5:8], [l[4]]]))) for l in sriped if len(l) > 1]
    return dict(dt)

def associate(first_list, second_list,offset,max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim
    to find the closest match for every input tuple.

    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))

    """
    first_keys = first_list.keys()
    second_keys = second_list.keys()
    potential_matches = [(abs(a - (b + offset)), a, b)
                         for a in first_keys
                         for b in second_keys
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))

    matches.sort()
    return matches

def align(model, data):
    """Align two trajectories using the method of Horn (closed-form).
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)

    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)

    """
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)

    W = np.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = np.linalg.linalg.svd(W.transpose())
    S = np.matrix(np.identity( 3 ))
    if(np.linalg.det(U) * np.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh

    rotmodel = rot*model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
	dots += np.dot(data_zerocentered[:,column].transpose(),rotmodel[:,column])
        normi = np.linalg.norm(model_zerocentered[:,column])
        norms += normi*normi

    s = float(dots / norms)
    return np.array(rot), s

def rigid_body_transform(quat,trans):
    T = transf.quaternion_matrix(quat)
    T[0:3,3] = trans
    return T

def plot_traj(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib.

    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend

    """
    stamps.sort()
    interval = np.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label)

def plot_translation_error(timestamps, translation_error, results_dir,name_postfix="",name_prefix=""):
    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(111, xlabel='time [s]', ylabel='position drift [mm]', xlim=[0,timestamps[-1]-timestamps[0]+4])
    ax.plot(timestamps-timestamps[0], translation_error[:,0]*1000, 'r-', label='x')
    ax.plot(timestamps-timestamps[0], translation_error[:,1]*1000, 'g-', label='y')
    ax.plot(timestamps - timestamps[0], translation_error[:, 2] * 1000, 'b-', label='z')
    lgd = ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
       ncol=3, mode="expand", borderaxespad=0.2)
    # ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir+'/'+name_prefix+'_translation_error_'+name_postfix+'.png',bbox_extra_artists=(lgd,), bbox_inches='tight')

def plot_rotation_error(timestamps, rotation_error, results_dir,name_postfix="",name_prefix=""):
    fig = plt.figure(figsize=(8, 4))
    # , xlim=[0,timestamps[-1]-timestamps[0]+9]
    ax = fig.add_subplot(111, xlabel='time [s]', ylabel='orientation drift [rad]', ylim=[-0.3,0.3])
    ax.plot(timestamps, rotation_error[:,0], 'r-', label='yaw')
    ax.plot(timestamps, rotation_error[:,1], 'g-', label='pitch')
    ax.plot(timestamps, rotation_error[:, 2], 'b-', label='roll')
    lgd = ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
       ncol=3, mode="expand", borderaxespad=0.2)
    # ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir+'/'+name_prefix+'_orientation_error_'+name_postfix+'.png', bbox_extra_artists=(lgd,), bbox_inches='tight')

def rpe_trajectory(gt_trajectory, es_trajectory,param_fixed_delta=False,param_delta=1.00,param_delta_unit="s",scale = 1):
    gt = [(key, evaluate_rpe.transform44(list(itertools.chain.from_iterable([[key], gt_trajectory[key][0:3],[gt_trajectory[key][6]],gt_trajectory[key][3:6]])))) for key in gt_trajectory]
    gt = dict(gt)

    es = [(key, evaluate_rpe.transform44(list(itertools.chain.from_iterable([[key], es_trajectory[key][0:3],[es_trajectory[key][6]],es_trajectory[key][3:6]])))) for key in es_trajectory]
    es=dict(es)
    result = np.array(evaluate_rpe.evaluate_trajectory(gt, es, param_fixed_delta=param_fixed_delta,param_delta=param_delta,param_delta_unit=param_delta_unit,param_scale=scale))
    stamps_rpe = result[:,0]
    trans_rpe_error = result[:,4]
    rot_rpe_error = result[:, 5]
    return stamps_rpe,trans_rpe_error,rot_rpe_error

def abs_trajectory(q_gt,p_gt,q_es,p_es):
    # use first 10 % frames to estimate 3d rotation, scale and translation of datapoints
    # init_frames = int(math.ceil(p_es.shape[0] / 10))
    init_frames = int(math.ceil(p_es.shape[0]))

    rot,scale = align(np.matrix(p_es[0:init_frames,:].T), np.matrix(p_gt[0:init_frames,:].T))

    trans = np.mean(p_gt, axis=0,keepdims=True).T - np.dot(scale*rot, np.mean(p_es, axis=0,keepdims=True).T)

    p_es_aligned = (np.dot( scale*rot, p_es.T) + trans).T

    T_gt = rigid_body_transform(q_gt[0,:], p_gt[0,:])
    T_es = rigid_body_transform(q_es[0,:], p_es_aligned[0,:])
    T_es_inv = np.linalg.inv(T_es)
    T_0 = np.dot(T_gt, T_es_inv)
    # print 'T_0 = ' + str(T_0)

    # apply transformation to estimated trajectory
    q_es_aligned = np.zeros(np.shape(q_es))
    rpy_es_aligned = np.zeros(np.shape(p_es))
    rpy_gt = np.zeros(np.shape(p_es))
    # orient_error = np.zeros(np.shape(p_es)[0])
    # orient_error_point = np.zeros(np.shape(p_es)[0])
    for i in range(np.shape(p_es)[0]):
        T_es = rigid_body_transform(q_es[i,:],p_es_aligned[i,:])
        T_gt = np.dot( T_0,T_es)
        q_es_aligned[i,:] = transf.quaternion_from_matrix(T_gt)
        # orient_error[i] = np.dot(q_es_aligned[i,:],transf.quaternion_inverse(q_gt[i,:]))
        rpy_es_aligned[i,:] = transf.euler_from_matrix(T_gt[0:3,0:3],'rzyx')
        rpy_gt[i,:] = transf.euler_from_quaternion(q_gt[i,:], 'rzyx')
        # orient_error = np.arccos(min(1, max(-1, (np.trace(T_gt[0:3, 0:3]) - 1) / 2)))
        # print(orient_error)
        # orient_error_point[i] = orient_error

    trans_error = (p_gt - p_es_aligned)
    trans_error_point = np.sqrt(np.sum(np.power(trans_error, 2), axis=1))


    orient_error_rpy = (rpy_gt - rpy_es_aligned)
    orient_error = np.sum(np.abs(orient_error_rpy),axis=1)
    #  normalize angle differences
    # orient_error = np.arctan2(np.sin(orient_error), np.cos(orient_error))
    # iner_prod = np.sum(np.multiply(q_es_aligned, q_gt), axis=1)
    # print(q_es_aligned,q_gt)
    # orient_error_point = 2 * np.arccos(np.abs(iner_prod))
    # orient_error_point -=  np.min(orient_error_point)
    # print(iner_prod[0],iner_prod.shape)
    # print("difference in errors:"+str(np.sum(trans_error2 - trans_error_point)))
    return p_es_aligned,trans_error_point,orient_error, scale

def normalize_quaternion(q):
    return  q / np.linalg.norm(q)

def metrics(err):
    data_dict = {}
    data_dict["rmse"] = float(np.sqrt(np.dot(err, err) / len(err)))
    data_dict["mean"] = float(np.mean(err))
    data_dict["median"] = float(np.median(err))
    data_dict["std"] = float(np.std(err))
    data_dict["min"] = float(np.min(err))
    data_dict["max"] = float(np.max(err))
    return data_dict


if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory.
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qw qx qy qz)')
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qw qx qy qz)')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    # parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', action='store_true' ,help='save associated files.')
    parser.add_argument('--plot', action='store_true', help='plot the first and the aligned second trajectory to an image')
    parser.add_argument('--results_dir', help='Existing directory where to save all outputs from this graph')
    parser.add_argument('--postfix', help='Name postfix used in all outputted filenames', default="")
    parser.add_argument('--prefix', help='Name prefix used in all outputted filenames', default="1")
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()

    scale = args.scale
    first_list = read_file_list(args.first_file)
    second_list = read_file_list(args.second_file)
    assoc_match_filename = args.results_dir+"/"+args.prefix+"_match_associated_"+args.postfix+".csv"
    if os.path.exists(assoc_match_filename):
        # load associations
       matches = read_file_list_raw(assoc_match_filename)
    else:
        # create associations
        matches = associate(first_list, second_list,float(args.offset),float(args.max_difference))
        if len(matches)<2:
            sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")

        if args.save_associations:
            file = open(assoc_match_filename,"w")
            file.write("\n".join(["%f,%f"%(a,b) for a,b in matches]))
            file.close()

    gt_matches =dict([(a,first_list[a]) for a, b in matches])
    es_matches = dict([(b, second_list[b]) for a, b in matches])

#  --------------ATE Calculation
    t_gt = np.array([float(a) for a, b in matches])
    q_gt = normalize_quaternion(np.array([[float(value) for value in first_list[a][3:7]] for a, b in matches]))
    p_gt = np.array([[float(value) for value in first_list[a][0:3]] for a, b in matches])

    t_es = np.array([float(b) for a, b in matches])
    q_es = normalize_quaternion(np.array([[float(value) for value in second_list[b][3:7]] for a, b in matches]))
    p_es = np.array([[float(value) for value in second_list[b][0:3]] for a,b in matches])

    # print(p_es)
    start_time = min(t_es[0], t_gt[0])
    t_es -= start_time
    t_gt -= start_time
    p_es_aligned, trans_abs_error, rot_abs_error,scale = abs_trajectory(q_gt, p_gt, q_es, p_es)
    # print(trans_rpe_error)
    # print(trans_abs_error)
    # print(rot_rpe_error)
    # print(rot_abs_error)

#    --------------------------------------------------
    #  --------------RPE Calculation
    stamps_rpe, trans_rpe_error, rot_rpe_error = (None,None,None)
    try:
        stamps_rpe, trans_rpe_error, rot_rpe_error = rpe_trajectory(gt_matches, es_matches, param_fixed_delta=True, param_delta=5, param_delta_unit='m',scale=scale)
    except:
        print("Unexpected error:", sys.exc_info()[0])
        stamps_rpe = np.array([0])
        trans_rpe_error = np.array([0])
        rot_rpe_error = np.array([0])

    to_deg = 180.0 / np.pi
    to_m = 100
    results = {}
    results["ATE"] = metrics(trans_abs_error*to_m)
    # results["absolute_orientation_error"] = metrics(rot_abs_error * to_deg)
    results["RPE_translation"] = metrics(trans_rpe_error * to_m)
    results["RPE_orientation"] = metrics(rot_rpe_error * to_deg)
    results["scale"] = scale
    results["RPE_trans_err"] = list(trans_rpe_error)
    results["RPE_rot_err"] = list(rot_rpe_error)

    results_json_file = args.results_dir+"/"+args.prefix+"_results."+args.postfix+".json"
    with open(results_json_file, "w") as f:
        f.write(json.dumps(results))

    if args.verbose:
        print "compared_pose_pairs %d pairs"%(len(trans_abs_error))
        # print(np.dot(trans_error_point,trans_error_point))
        print(results["ATE"])
        # print(results["absolute_orientation_error"])
        print("\nRPE")
        # print "compared_pose_pairs %d pairs"%(len(trans_rpe_error))
        print(results["RPE_translation"])
        print( results["RPE_orientation"])



    # if args.save:
    #     file = open( args.results_dir+"/,"w")
    #     file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps,second_xyz_full_aligned.transpose().A)]))
    #     file.close()

    if args.plot:

        # plot position error (drift)
        # print(trans_rpe_error)
        # plot_translation_error(t_es, trans_rpe_error, args.results_dir,args.postfix,args.prefix)

        # plot orientation error (drift)
        # plot_rotation_error(t_es, rot_rpe_error, args.results_dir,args.postfix,args.prefix)
        left, width = .05, .5
        bottom, height = .05, .5
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(ax,t_gt,p_gt,'-',"black","ground truth")
        plot_traj(ax,t_es,p_es_aligned,'-',"green","estimated")
        ax.text(left, bottom, 'scale: '+str(scale),
                    horizontalalignment='left',
                    verticalalignment='top',
                    transform=ax.transAxes)
        # label="difference"
        # for (x1,y1,z1),(x2,y2,z2) in zip(p_gt,p_es_aligned):
        #     ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
        #     label=""

        ax.legend()

        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.savefig( args.results_dir+"/"+args.prefix+"_trajectory_"+args.postfix+".png",dpi=90)


