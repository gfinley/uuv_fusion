#!/usr/bin/env python
import networkx as nx
import time
from copy import deepcopy
import matplotlib.pyplot as plt
from heapq import heappush, heappop
from itertools import count
from networkx import NetworkXError
import networkx as nx
import sys
import os.path
import yaml
import rospy
import numpy as np
from itertools import count
from scipy.stats.stats import tvar
#from nav_msgs.msg import Path
from cavr_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, Polygon, Point32, Point
from cavr_msgs.msg import MapFeature, TopoFeature

class Feature:
    def __init__(self):
        self.id = 0
        self.frame_id = ''
        self.ref_frame_id = ''
        self.name = ''
        self.type = ''
        self.shape = ''
        self.description = ''
        self.location = []
        self.orientation = []
        self.altitude = 0.0
        self.depth = 0.0
        self.height = 0.0
        self.radius = 0.0
        self.geometry = []
        self.n = []
        self.e = []
        self.s = []
        self.w = []
        
    def populateFeature(self, id, data):
        self.id = id
        self.frame_id = data['frame_id']
        self.ref_frame_id = data['ref_frame_id']
        
        if 'obst' in data['type']:
            self.name = 'obst_%03d' % id
            self.type = data['type']
            self.location = deepcopy(data['location'])
            self.altitude = data['altitude']
            self.depth = data['depth']
            self.height = data['height']
            self.orientation = deepcopy(data['orientation'])
            if 'circle' in data['shape']:
                self.radius = data['radius']
            elif 'poly' in data['shape']:
                self.geometry = deepcopy(data['geometry'])
                self.radius = 0.0
                for g in self.geometry:
                    radius = (g[0]**2 + g[1]**2)**0.5
                    if radius > self.radius:
                        self.radius = radius
                # check if the first and last points of polygon overlap. If not, add that segment
                if len(self.geometry)>2 and not self.geometry[0] == self.geometry[-1]:
                    self.geometry.append(self.geometry[0])        
                
            self.northernPoint()
            self.easternPoint()
            self.southernPoint()
            self.westernPoint()
        elif 'boundary' in data['type']:
            self.name = 'box'
            self.type = data['type']
            self.n = data['x_max'] 
            self.e = data['y_max'] 
            self.s = data['x_min'] 
            self.w = data['y_min']
        elif 'nav_feature' in data['type']:
            self.name = 'nav_%03d' % data['id']
            self.id = data['id']
            self.type = data['type']
            self.shape = data['shape']
            self.location = deepcopy(data['location'])
            self.orientation = deepcopy(data['orientation'])
            self.altitude = data['altitude']
            self.depth = data['depth']
            self.height = data['height']
            self.radius = data['radius']
            # self.northernPoint()
            # self.easternPoint()
            # self.southernPoint()
            # self.westernPoint()            
 
    def northernPoint(self):
        if 'circle' in self.type:
            self.n = (self.location[0]+self.radius, self.location[1])
        elif 'poly' in self.type:
            x_max = -1e16
            y_max = None
            for p in self.geometry:
                if self.location[0]+p[0]>x_max:
                    x_max = self.location[0]+p[0]
                    y_max = self.location[1]
            self.n = (x_max, y_max)
        else:
            #for nav_features
            self.n = (self.location[0]+self.radius, self.location[1])
            
    def easternPoint(self):
        if 'circle' in self.type:
            self.e = (self.location[0], self.location[1]+self.radius)
        elif 'poly' in self.type:
            x_max = -1e16
            y_max = -1e16
            for p in self.geometry:
                if self.location[1]+p[1]>y_max:
                    x_max = self.location[0]
                    y_max = self.location[1]+p[1]
            self.e = (x_max, y_max)
        else:
            self.e = (self.location[0], self.location[1]+self.radius)

    def southernPoint(self):
        if 'circle' in self.type:
            self.s = (self.location[0]-self.radius, self.location[1])
        elif 'poly' in self.type:
            x_max = 1e16
            y_max = 1e16
            for p in self.geometry:
                if self.location[0]+p[0]<x_max:
                    x_max = self.location[0]+p[0]
                    y_max = self.location[1]
            self.s = (x_max, y_max)
        else:
            self.s = (self.location[0]-self.radius, self.location[1])

    def westernPoint(self):
        if 'circle' in self.type:
            self.w = (self.location[0], self.location[1]-self.radius)
        elif 'poly' in self.type:
            x_max = 1e16
            y_max = 1e16
            for p in self.geometry:
                if self.location[1]+p[1]<y_max:
                    x_max = self.location[0]
                    y_max = self.location[1]+p[1]
            self.w = (x_max, y_max)
        else:
            self.w = (self.location[0], self.location[1]-self.radius)
        
def topoToString(topo):
    s = ''
    for t in topo:
        if s == '':
            s = t
        else:
            s = s+'-%s'%t
    return s

def stringToTopo(s):
    t = []
    if s=='':
        t = ['']
    else:        
        fields = s.split('-')
        for f in fields:
            t.append(f)
    return t

def drawEnvironment(env):
    fig = plt.gcf()
    for k, f in env.iteritems():
        if f.name == 'box':
            plt.plot([f.w, f.e],[f.n, f.n],'g')
            plt.plot([f.e, f.e],[f.n, f.s],'g')
            plt.plot([f.e, f.w],[f.s, f.s],'g')
            plt.plot([f.w, f.w],[f.s, f.n],'g')
        else:
            circle = plt.Circle((f.location[1], f.location[0]), f.radius, color='r', fill=False)
            fig.gca().add_artist(circle)
            if 'poly' in f.type:
                x = []
                y = []
                for g in f.geometry:
                    x.append(f.location[0]+g[0])
                    y.append(f.location[1]+g[1])
                if not x == []:
                    x.append(x[0])
                    y.append(y[0])
                plt.plot(y,x,'r')
    return

def drawBeams(beams, topos, direction=None):
    for k, b in beams.iteritems():
        plt.plot(b[1], b[0])
        if direction == 'positive':
            plt.text((b[1][0]+b[1][1])/2, (b[0][0]+b[0][1])/2, topoToString([topos[k][0]]))
        elif direction == 'negative':
            plt.text((b[1][0]+b[1][1])/2, (b[0][0]+b[0][1])/2, topoToString([topos[k][1]]))
    
    return
   
def loadEnvironment(filename):
    env = {} # dictionary to be populated
    envFrameIds = {}
    
    if os.path.isfile(filename):
        g_l = []
        # grab the data
        f = open(filename)
        for data in yaml.safe_load_all(f):
            g_l.append(data)
        f.close()
        
        if len(g_l)>0:
            # There were actually goals in the list
            rospy.loginfo("%d environment features specified in %s", len(g_l), filename)

            # process the environment features
            id = 0
            for g in g_l:
                if g['type'] == 'boundary':
                    box = Feature()
                    box.populateFeature(id, g)
                    id += 1
                    env[box.name] = box
                    envFrameIds[box.name] = box.frame_id
                elif g['type'] == 'obst_circle':
                    circle = Feature()
                    circle.populateFeature(id, g)
                    id += 1
                    env[circle.name] = circle
                    envFrameIds[circle.name] = circle.frame_id
                elif g['type'] == 'obst_poly':
                    poly = Feature()
                    poly.populateFeature(id, g)
                    id += 1
                    env[poly.name] = poly
                    envFrameIds[poly.name] = poly.frame_id
                elif g['type'] == 'nav_feature':
                    nav_feature = Feature()
                    nav_feature.populateFeature(id,g)
                    id +=1
                    env[nav_feature.name] = nav_feature
                    envFrameIds[nav_feature.name] = nav_feature.frame_id
                elif g['type'] == 'transponder':
                    nav_feature = Feature()
                    nav_feature.populateFeature(id,g)
                    id +=1
                    env[nav_feature.name] = nav_feature
                    envFrameIds[nav_feature.name] = nav_feature.frame_id
                else:
                    rospy.logwarn("Environment feature %s not recognized", g.type)
                    continue

        else:
            rospy.loginfo("NO features specified in %s. IGNORING request.", len(g_l), filename)
        
    else:
        rospy.logwarn("File %s does not exist. Ignoring LOAD ENVIRONMENT request.", filename)
        return None

    return env
   
def createTopo(env, obs, direction, count, pos, neg):
    box = env['box']
    xo = obs.location[0]
    yo = obs.location[1]
    xb = xo
    yb = yo
    
    topo = TopoFeature()
    topo.header.frame_id = obs.frame_id
    point = Point()
    point.x = obs.location[0]
    point.y = obs.location[1]
    point.z = 0.0
    topo.beam.append(point)
    topo.pos = '%s%03d'%(pos, count)
    topo.neg = '%s%03d'%(neg, count)

    # assume the beam goes all the way to the box
    if direction == 'north':
        topo.name = obs.name+'-n'
        xb = box.n
        yb = yo
    elif direction == 'east':
        topo.name = obs.name+'-e'
        xb = xo
        yb = box.e
    elif direction == 'south':
        topo.name = obs.name+'-s'
        xb = box.s
        yb = yo
    elif direction == 'west':
        topo.name = obs.name+'-w'
        xb = xo
        yb = box.w

    # now cycle through other obstacles to see if there is an intersect
    for ki, fi in env.iteritems():
        if 'box' in fi.name:
            continue
        if 'nav_feature' in fi.name:
            continue
        if obs.name == fi.name:
            continue

        if direction == 'north':
            # ignore if it lies to the south
            if fi.location[0] < xo:
                continue
            
            # ignore if it lies to the east or west
            if yo > fi.w[1] and yo < fi.e[1]:
                # intersect!
                if fi.location[0] < xb:
                    # shorter, so use this as new beam end point
                    xb = fi.location[0]
        elif direction == 'south':
            # ignore if it lies to the north
            if fi.location[0] > xo:
                continue
            
            # ignore if it lies to the east or west
            if yo > fi.w[1] and yo < fi.e[1]:
                # intersect!
                if fi.location[0] > xb:
                    # shorter, so use this as new beam end point
                    xb = fi.location[0]
        elif direction == 'east':
            # ignore if it lies to the west
            if fi.location[1] < yo:
                continue
            
            # ignore if it lies to the east or west
            if xo > fi.s[0] and xo < fi.n[0]:
                # intersect!
                if fi.location[1] < yb:
                    # shorter, so use this as new beam end point
                    yb = fi.location[1]
        elif direction == 'west':
            # ignore if it lies to the east
            if fi.location[1] > yo:
                continue
            
            # ignore if it lies to the east or west
            if xo > fi.s[0] and xo < fi.n[0]:
                # intersect!
                if fi.location[1] > yb:
                    # shorter, so use this as new beam end point
                    yb = fi.location[1]
    point = Point()
    point.x = xb
    point.y = yb
    point.z = 0.0
    topo.beam.append(point)

    return topo
    
def generateTopos(env, v_sub='v', V_sub='V', h_sub='h', H_sub='H'):
    topos = {}
    count = 0
    for k, f in env.iteritems():
        if 'box' in f.name:
            continue
            t = createTopo(env, f, 'north', count, v_sub, V_sub)
            topos[t.name] = t
            t = createTopo(env, f, 'east', count, h_sub, H_sub)
            topos[t.name] = t
            count += 1
            t = createTopo(env, f, 'south', count, v_sub, V_sub)
            topos[t.name] = t
            t = createTopo(env, f, 'west', count, h_sub, H_sub)
            topos[t.name] = t
            count += 1
        
    # finally, make sure that there are no repetitions of beams
    toDel = []
    for k, t in topos.iteritems():
        for ki, ti in topos.iteritems():
            if k == ki:
                continue
            if (t.beam[0] == ti.beam[0] and t.beam[1] == ti.beam[1]) or (t.beam[0] == ti.beam[1] and t.beam[1] == ti.beam[0]):
                if not (k, ki) in toDel and not (ki, k) in toDel:
                    toDel.append((k, ki))
    for d in toDel:
        print 'Deleting %s' % d[0]
        del topos[d[0]]
        
    return topos
    
def convertMapFeatureToFeature(mf):
    f = Feature()
    f.frame_id = mf.header.frame_id
    f.ref_frame_id = mf.ref_frame_id
    f.name = mf.name
    f.id = mf.id
    f.type = mf.type_str
    f.radius = mf.radius
    f.altitude = mf.altitude
    f.height = mf.height
    if len(mf.n) > 0:
        f.n = [mf.n[0], mf.n[1]]
        f.e = [mf.e[0], mf.e[1]]
        f.s = [mf.s[0], mf.s[1]]
        f.w = [mf.w[0], mf.w[1]]
    f.location = [mf.pose.pose.position.x, mf.pose.pose.position.y]
    f.depth = mf.pose.pose.position.z
    for p in mf.geometry.points:
        f.geometry.append([p.x, p.y])
    return f

def convertFeatureToMapFeature(f):
    mf = MapFeature()
    mf.header.frame_id = f.frame_id
    mf.ref_frame_id = f.ref_frame_id
    mf.name = f.name
    mf.id = f.id
    mf.type_str = f.type
    mf.radius = f.radius
    mf.altitude = f.altitude
    mf.height = f.height
    if not f.n == []:
        mf.n.append(f.n[0])
        mf.n.append(f.n[1])
        mf.e.append(f.e[0])
        mf.e.append(f.e[1])
        mf.s.append(f.s[0])
        mf.s.append(f.s[1])
        mf.w.append(f.w[0])
        mf.w.append(f.w[1])
    mf.pose = PoseWithCovariance()
    mf.pose.pose.position.x = f.location[0]
    mf.pose.pose.position.y = f.location[1]
    mf.pose.pose.position.z = f.depth
    mf.pose.pose.orientation.x = f.orientation[0]
    mf.pose.pose.orientation.y = f.orientation[1]
    mf.pose.pose.orientation.z = f.orientation[2]    
    mf.pose.pose.orientation.w = f.orientation[3]
    mf.geometry = Polygon()
    for g in f.geometry:
        p = Point32()
        p.x = g[0]
        p.y = g[1]
        p.z = f.depth
        mf.geometry.points.append(p)
    return mf
