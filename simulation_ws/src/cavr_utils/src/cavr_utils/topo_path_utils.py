#!/usr/bin/env python
import networkx as nx
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
import cavr_utils.feature_utils as feature_utils
from geometry_msgs.msg import PoseStamped
from collections import OrderedDict
from copy import deepcopy

class nodeNamer:
    def __init__(self, s, c=0):
        self.prefix = s
        self.count = c
    
    def getUniqueName(self):
        name = self.prefix+'%03d'%self.count
        self.count += 1
        return name

def simplifyTopo(t_in, t_s, identitySeq):
    t_out = []
    # currently no topological considerations
    if t_in == ['']:
        return deepcopy(t_s)
    
    # this segment does modify the topological constraints
    if t_s == ['']:
        return deepcopy(t_in)
    
    # there is actually something to check here
    t_in_l = deepcopy(t_in)
    t_s_l = deepcopy(t_s)
    while len(t_s_l)>0 and len(t_in_l)>0:
        if t_s_l[0] == identitySeq[t_in_l[-1]]:
            t_s_l = t_s_l[1:] # first element gets cancelled
            t_in_l = t_in_l[:-1] # last element gets cancelled
        else:
            break
    
    if len(t_in_l) == 0:
        if len(t_s_l) == 0:
            return ['']
        else:
            return t_s_l
    
    t_in_l.extend(t_s_l)
    return t_in_l

def topoToCost(topo):
    return 1*len(topo)

def distanceBetweenNodes(n1, n2):
    return ((n1[0]-n2[0])**2 + (n1[1]-n2[1])**2)**0.5 

def drawGraph(G, direction='positive', displayEdges=True, displayTopo = None):
    # draw the nodes
    states = nx.get_node_attributes(G, 'state') # dictionary of node states
    for key in states:
        (x,y) = states[key]
        plt.plot(y,x,'ro')
        if 'veh' in key or 'wpt' in key:
            plt.text(y,x,key)
        else:
            plt.text(y,x,key[-3:])
    # plot the edges
    edges = nx.edges(G)
    for e in edges:
        (x0, y0) = states[e[0]]
        (x1, y1) = states[e[1]]
        if direction=='positive' and y0 > y1:
            continue
        elif direction=='negative' and y0 < y1:
            continue
        if displayEdges:
            plt.plot([y0, y1], [x0, x1], 'k')
        if not displayTopo == None:
            tv = feature_utils.topoToString(G[e[0]][e[1]][0]['tv'])
            th = feature_utils.topoToString(G[e[0]][e[1]][0]['th'])
            if 'vert' in displayTopo:
                plt.text(0.5*(y0+y1), 0.5*(x0+x1), tv)
            elif 'hor' in displayTopo:
                plt.text(0.5*(y0+y1), 0.5*(x0+x1), th)
            elif 'both' in displayTopo:
                plt.text(0.5*(y0+y1), 0.5*(x0+x1), tv+'+'+th)
    return

def drawPath(G, path1, path2, cost, box):
    states = nx.get_node_attributes(G, 'state') # dictionary of node states
    x = []
    y = []
    f = path1[0].split('_')
    t_in = [f[1], f[2]] # format is: name_tv1-tv2_th1-th2
    if t_in[0] == '':
        t_in[0] = 'NONE'
    if t_in[1] == '':
        t_in[1] = 'NONE'
    for p in path1:
        f = p.split('_')
        (x_, y_) = states[f[0]]
        x.append(x_)
        y.append(y_)
        t_out = [f[1], f[2]] # format is: node_v_tv1-tv2_h_th1-th2
    for p in path2:
        f = p.split('_')
        (x_, y_) = states[f[0]]
        x.append(x_)
        y.append(y_)
        t_out = [f[1], f[2]] # format is: node_v_tv1-tv2_h_th1-th2
    if t_out[0] == '':
        t_out[0] = 'NONE'
    if t_out[1] == '':
        t_out[1] = 'NONE'
    plt.plot(y,x,'b',linewidth=2.0)
    plt.text(box.w+1,box.s+4,'Topo in = '+t_in[0]+' and '+t_in[1])
    plt.text(box.w+1,box.s+3,'Topo out = '+t_out[0]+' and '+t_out[1])
    plt.text(box.w+1,box.s+2,'Path cost = %f' % cost)
    return

def astar_path_2segment(G, source, intermediate, target, topo, a1, a2, identitySeq, addFinalNode):
    path1, cost, topo1 = astar_path(G, source, intermediate, topo, a1, a2, identitySeq)
    path2 = []
    topo2 = deepcopy(topo1)
    if addFinalNode:
        path2, cost2, topo2 = astar_path(G, intermediate, target, topo1, a1, a2, identitySeq)
        cost += cost2
    return path1, path2, cost, topo1, topo2

def astar_path(G, source, target, topo, a1, a2, identitySeq):
    #print a1, a2
    push = heappush
    pop = heappop
    
    # The queue stores priority, counter, node, topo, g_dist, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guaranteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), source, topo, 0, None)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {} # (g_cost, h_cost)
    # Maps explored nodes to parent closest to the source.
    explored = {} # (parent, g_cost)

    states = nx.get_node_attributes(G, 'state') # dictionary of node states
    
    while queue:
        # Pop the smallest item from queue.
        priority, __, curnode, topo, g_cost, parent_topo = pop(queue)
        tv_in = topo[0]
        th_in = topo[1]
        
        curnode_topo = '_'.join([curnode, feature_utils.topoToString(tv_in), feature_utils.topoToString(th_in)]) 

        if curnode == target:
            path = [curnode_topo]
            while parent_topo is not None:
                path.append(parent_topo)
                parent_topo = explored[parent_topo][0]
            path.reverse()
            return path, g_cost, topo

        if curnode_topo in explored:
            if explored[curnode_topo][1] > g_cost:
                print "WARNING: Node %s has cost %f, which is greater than new path cost %f" % (node, g_cost, explored[node][1])
            continue

        explored[curnode_topo] = (parent_topo, g_cost)
        # Grab all the neighbors
        for n in G.neighbors(curnode):
            # Calculate the topological string to reach this neighbor
            tv_s = G[curnode][n][0]['tv']
            tv_out = simplifyTopo(tv_in, tv_s, identitySeq)
            #print 'tv_out = ', tv_out, ' tv_in = ', tv_in, ' tv_s = ', tv_s
            th_s = G[curnode][n][0]['th']
            #print curnode_topo, ' th_in = ', th_in, ' th_s = ', th_s
            th_out = simplifyTopo(th_in, th_s, identitySeq)
            #print curnode_topo, ' th_out = ', th_out, ' th_in = ', th_in, ' th_s = ', th_s
            neighbor = '_'.join([n, feature_utils.topoToString(tv_out), feature_utils.topoToString(th_out)])
            # Calculate the cost to reach neighbor (path distance only)
            n_cost = g_cost+a1*distanceBetweenNodes(states[curnode], states[n]) + a2*topoToCost(tv_out) + a2*topoToCost(th_out)
            if neighbor in explored:
                if explored[neighbor][1] > n_cost:
                    print "WARNING: Node %s had cost %f, which is greater than new path cost %f" % (neighbor, p_cost, explored[neighbor][1])
                    continue
            if neighbor in enqueued:
                # already handled, so get previous costs
                q_cost, h_cost = enqueued[neighbor]
                # if qcost < ncost, a longer path to neighbor remains
                # enqueued. Removing it would need to filter the whole
                # queue, it's better just to leave it there and ignore
                # it when we visit the node a second time.
                #print 'IGNORED: ', curnode_topo, neighbor, n_cost + h_cost, p_cost, p_cost_
                if q_cost <= n_cost:
                    #print 'Already reached %s with cost %f instead of %f' % (neighbor, q_cost+h_cost, n_cost+h_cost)
                    continue
            else:
                h_cost = distanceBetweenNodes(states[n], states[target]) # straight line distance as heuristic
            enqueued[neighbor] = n_cost, h_cost
            push(queue, (n_cost + h_cost, next(c), n, [tv_out, th_out], n_cost, curnode_topo))
            #print curnode_topo, neighbor, n_cost + h_cost, p_cost

    raise nx.NetworkXNoPath("Node %s not reachable from %s" % (source, target))    
    
def calcTopos(s1, s2, topos):
    tv = [] # vertical
    th = [] # horizontal
    x1 = s1[0]
    y1 = s1[1]
    x2 = s2[0]
    y2 = s2[1]
    xp = None
    yp = None

    # we first need to sort the topos
    vs = {}
    hs = {}
    for k,t in topos.iteritems():
        if '-n' in t.name or '-s' in t.name:
            vs[t.name] = abs(t.beam[0].y - y1) # both beam points have same y
            #vs[t.name] = t.beam[0].y # both beam points have same y
        elif '-e' in t.name or '-w' in t.name:
            hs[t.name] = abs(t.beam[0].x - x1) # both beam points have same x
            #hs[t.name] = t.beam[0].x # both beam points have same x
    # now sort these dictionaries by their values
    v_sort = OrderedDict(sorted(vs.items(), key=lambda t: t[1]))
    h_sort = OrderedDict(sorted(hs.items(), key=lambda t: t[1]))
    
    # handle vertical beams
    for i in range(len(v_sort)):
        (k, v) = v_sort.popitem(False)
        t = topos[k]
        xb_s = t.beam[0].x  # start
        yb_s = t.beam[0].y
        xb_f = t.beam[1].x  # finish
        yb_f = t.beam[1].y

        # dealing with vertical beam -> yb_s == yb_f
        if yb_s >= y1 and yb_s < y2:
            # left to right
            if abs(y2-y1) < 1e-6:
                xp = x1
            else:
                xp = x1+(yb_s-y1)*(x2-x1)/(y2-y1)
            if '-n' in k and xp >= xb_s and xp <= xb_f:
                tv.append(t.pos)
            elif '-s' in k and xp >= xb_f and xp <= xb_s:
                tv.append(t.pos)
                
        elif yb_s >= y2 and yb_s < y1:
            # right to left
            if abs(y2-y1) < 1e-6:
                xp = x2
            else:
                xp = x2+(yb_s-y2)*(x1-x2)/(y1-y2)
            if '-n' in k and xp >= xb_s and xp <= xb_f:
                tv.append(t.neg)
            elif '-s' in k and xp >= xb_f and xp <= xb_s:
                tv.append(t.neg)

    # handle horizontal beams
    for i in range(len(h_sort)):
        (k, h) = h_sort.popitem(False)
        t = topos[k]
        xb_s = t.beam[0].x  # start
        yb_s = t.beam[0].y
        xb_f = t.beam[1].x  # finish
        yb_f = t.beam[1].y

        # dealing with horizontal beam -> xb_s == xb_f
        if xb_s >= x1 and xb_s < x2:
            # south to north
            if abs(x2-x1) < 1e-6:
                yp = y1
            else:
                yp = y1+(xb_s-x1)*(y2-y1)/(x2-x1)
            if '-e' in k and yp >= yb_s and yp <= yb_f:
                th.append(t.pos)
            elif '-w' in k and yp >= yb_f and yp <= yb_s:
                th.append(t.pos)
        elif xb_s >= x2 and xb_s < x1:
            # north to south
            if abs(x2-x1) < 1e-6:
                yp = y2
            else:
                yp = y2+(xb_s-x2)*(y1-y2)/(x1-x2)
            if '-e' in k and yp >= yb_s and yp <= yb_f:
                th.append(t.neg)
            elif '-w' in k and yp >= yb_f and yp <= yb_s:
                th.append(t.neg)
    
    if tv == []:
        tv = ['']
    if th == []:
        th = ['']
    return (tv, th)

def generateRoadmap(env, topos, namer, frame_id):
    # create a graph
    MG = nx.MultiDiGraph()
    MG.graph['frame_id'] = frame_id
    # add nodes around the obstacles
    offset = 0.2
    for k,f in env.iteritems():
        if k == 'box':
            continue
        
        # add nodes n, e, s, and w of the obstacle
        name = namer.getUniqueName()
        MG.add_node(name)
        MG.node[name]['state'] = [f.n[0] + f.radius*1.1, f.n[1]+offset] # north
        name = namer.getUniqueName()
        MG.add_node(name)
        MG.node[name]['state'] = [f.e[0]+offset, f.e[1] + f.radius*1.1] # east
        name = namer.getUniqueName()
        MG.add_node(name)
        MG.node[name]['state'] = [f.s[0] - f.radius*1.1, f.s[1]+offset] #south
        name = namer.getUniqueName()
        MG.add_node(name)
        MG.node[name]['state'] = [f.w[0]+offset, f.w[1] - f.radius*1.1] # west
    
    states = nx.get_node_attributes(MG, 'state')
    # make sure all the nodes are collision free
#     col_nodes = []
#     for kn,n in states.iteritems():
#         for kf, f in env.iteritems():
#             # project the current feature onto the segment
#             if f.name == 'box':
#                 continue
#             o = np.array([f.location[0]-n[0], f.location[1]-n[1]])
#             if np.linalg.norm(o) < f.radius:
#                 col_nodes.append(kn)
#                 break
#     rospy.loginfo('%d nodes in collision state, so removing', len(col_nodes))
#     for n in col_nodes:
#         MG.remove_node(n)
    
    # Add edges if collision free
    for kn,n in states.iteritems():
        for kni,ni in states.iteritems():
            if kn == kni:
                continue
            
            # check if we are in collision
            if not inCollision(n, ni, env):
                # no collision occurred for this segment, so add the edge
                # first calculate the topographical information for this segment
                
                (tv, th) = calcTopos(n, ni, topos)
                MG.add_edge(kn, kni, tv=tv, th=th)
                #print kn, kni, tv, th
                
    return MG

def inCollision(s1, s2, env):
    collide = False
    
    if s1 == s2:
        # no segment, just a point
        for kf, f in env.iteritems():
            # project the current feature onto the segment
            if f.name == 'box':
                continue
            
            o = np.array([f.location[0]-s1[0], f.location[1]-s1[1]])
            
            if np.linalg.norm(o) < f.radius:
                # collide!
                collide = True
                break
        
    else:
        
        # define the segment
        s = np.array([s2[0]-s1[0], s2[1]-s1[1]])
        sn = np.linalg.norm(s)
        su = s/sn
        
        for kf, f in env.iteritems():
            # project the current feature onto the segment
            if f.name == 'box':
                continue
            
            o = np.array([f.location[0]-s1[0], f.location[1]-s1[1]])
            
            po = np.dot(o,su)
            if po < 0:
                # before start
                p = 0*s
            elif po > sn:
                # after end
                p = s
            else:
                # on segment
                p = po*su
            # calc the offset
            d = p - o
            
            if np.linalg.norm(d) < f.radius:
                # collide!
                collide = True
                break
        
    return collide
    
def augmentRoadmap(G, source, target, env, topos):
    # add the source as a node to the graph
    nodes = {}
    if 'wpt' in source[2]:
        if not source[2] in G.nodes():
            # add this waypoint as a node
            G.add_node(source[2])
            G.node[source[2]]['state'] = [source[0], source[1]]
            nodes[source[2]] = [source[0], source[1]]
    else:
        # add this position as a node (e.g., veh current position)
        if source[2] in G.nodes():
            #print G.edges()
            # remove this node (and all related edges)
            G.remove_node(source[2]) # update
            #print G.edges()
        # add node and define state
        G.add_node(source[2])
        G.node[source[2]]['state'] = [source[0], source[1]]
        nodes[source[2]] = [source[0], source[1]]
    
    # now handle the target
    if not target[2] in G.nodes():
        # add this waypoint as a node
        G.add_node(target[2])
        G.node[target[2]]['state'] = [target[0], target[1]]
        nodes[target[2]] = [target[0], target[1]]
    
    # update the edges
    states = nx.get_node_attributes(G, 'state')

    for kn, n in nodes.iteritems():
        for kni, ni in states.iteritems():
            if kn == kni:
                continue
            
            # check if we are in collision
            if not inCollision(n, ni, env):
                # no collision occurred for this segment, so add the edges in both directions
                (tv, th) = calcTopos(n, ni, topos)
                G.add_edge(kn, kni, tv=tv, th=th)
                if not kni in nodes:
                    (tv, th) = calcTopos(ni, n, topos)
                    G.add_edge(kni, kn, tv=tv, th=th)
    
    return G, source[2], target[2] 
    
def augmentRoadmapGoal(G, goal, env, topos, addFinalNode):
    # add the source as a node to the graph
    nodes = {}
    if 'goal' in goal.pose_init_name:
        if not goal.pose_init_name in G.nodes():
            # add this waypoint as a node
            G.add_node(goal.pose_init_name)
            G.node[goal.pose_init_name]['state'] = [goal.pose_init.position.x, goal.pose_init.position.y]
            nodes[goal.pose_init_name] = [goal.pose_init.position.x, goal.pose_init.position.y]
        elif int(goal.pose_init_name[-3:])<10:
            # special goal, so update
            G.remove_node(goal.pose_init_name) # update
            G.node[goal.pose_init_name]['state'] = [goal.pose_init.position.x, goal.pose_init.position.y]
            nodes[goal.pose_init_name] = [goal.pose_init.position.x, goal.pose_init.position.y]
    else:
        # add this position as a node (e.g., veh current position)
        if goal.pose_init_name in G.nodes():
            # remove this node (and all related edges)
            G.remove_node(goal.pose_init_name) # update
        # add node and define state
        G.add_node(goal.pose_init_name)
        G.node[goal.pose_init_name]['state'] = [goal.pose_init.position.x, goal.pose_init.position.y]
        nodes[goal.pose_init_name] = [goal.pose_init.position.x, goal.pose_init.position.y]
    
    # now handle the intermediate point
    if not goal.name_1 in G.nodes():
        # add this waypoint as a node
        G.add_node(goal.name_1)
        G.node[goal.name_1]['state'] = [goal.pose_1.position.x, goal.pose_1.position.y]
        nodes[goal.name_1] = [goal.pose_1.position.x, goal.pose_1.position.y]
    elif int(goal.name_1[-3:])<10:
        # special goal, so update
        G.remove_node(goal.name_1) # update
        G.add_node(goal.name_1)
        G.node[goal.name_1]['state'] = [goal.pose_1.position.x, goal.pose_1.position.y]
        nodes[goal.name_1] = [goal.pose_1.position.x, goal.pose_1.position.y]

        # distinguish between a station-keep (zero velocity) and via point
    if addFinalNode:
        # now handle the final point
        if not goal.name_2 in G.nodes():
            # add this waypoint as a node
            G.add_node(goal.name_2)
            G.node[goal.name_2]['state'] = [goal.pose_2.position.x, goal.pose_2.position.y]
            nodes[goal.name_2] = [goal.pose_2.position.x, goal.pose_2.position.y]
        elif int(goal.name_1[-3:])<10:
            # special goal, so update
            G.remove_node(goal.name_2) # update
            G.add_node(goal.name_2)
            G.node[goal.name_2]['state'] = [goal.pose_2.position.x, goal.pose_2.position.y]
            nodes[goal.name_2] = [goal.pose_2.position.x, goal.pose_2.position.y]
    
    # update the edges
    states = nx.get_node_attributes(G, 'state')

    for kn, n in nodes.iteritems():
        for kni, ni in states.iteritems():
            if kn == kni:
                continue
            
            # check if we are in collision
            if not inCollision(n, ni, env):
                # no collision occurred for this segment, so add the edges in both directions
                (tv, th) = calcTopos(n, ni, topos)
                G.add_edge(kn, kni, tv=tv, th=th)
                if not kni in nodes:
                    (tv, th) = calcTopos(ni, n, topos)
                    G.add_edge(kni, kn, tv=tv, th=th)
    
    return G, goal.pose_init_name, goal.name_1, goal.name_2
    
def packagePath(G, path1_n, path2_n, frame_id):
    path = Path()
    path.poses_leg1 = []
    path.poses_leg2 = []
    states = nx.get_node_attributes(G, 'state')
    # first segment
    for p in path1_n:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.orientation.w = 1.0
        f = p.split('_')
        (pose.pose.position.x, pose.pose.position.y) = states[f[0]]
        path.poses_leg1.append(pose)
        
    # second segment
    for p in path2_n:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.orientation.w = 1.0
        f = p.split('_')
        (pose.pose.position.x, pose.pose.position.y) = states[f[0]]
        path.poses_leg2.append(pose)
    return path

def generateIdentitySeq(topos):
    identitySeq = {}
    for k,t in topos.iteritems():
        identitySeq[t.pos] = t.neg
        identitySeq[t.neg] = t.pos
    return identitySeq

def calcToposPoses(p1, p2, topos):
    tv = [] # vertical
    th = [] # horizontal
    x1 = y1 = x2 = y2 = xp = yp = None
    #print v_sort
    #print h_sort

    # we first need to sort the topos
    vs = {}
    hs = {}
    for k,t in topos.iteritems():
        if '-n' in t.name or '-s' in t.name:
            vs[t.name] = abs(t.beam[0].y - p1[t.header.frame_id].pose.position.y) # both beam points have same y
            #vs[t.name] = t.beam[0].y # both beam points have same y
        elif '-e' in t.name or '-w' in t.name:
            hs[t.name] = abs(t.beam[0].x - p1[t.header.frame_id].pose.position.x) # both beam points have same x
            #hs[t.name] = t.beam[0].x # both beam points have same x
    # now sort these dictionaries by their values
    v_sort = OrderedDict(sorted(vs.items(), key=lambda t: t[1]))
    h_sort = OrderedDict(sorted(hs.items(), key=lambda t: t[1]))


    # handle vertical beams
    for i in range(len(v_sort)):
        (k, v) = v_sort.popitem(False)
        t = topos[k]
        #print t.header.frame_id in p1
        
        # do we have the required information available?
        if not t.header.frame_id in p1:
            rospy.logwarn('Frame %s not recognized. Continuing', t.ref_frame_id)
            continue
        
        x1 = p1[t.header.frame_id].pose.position.x
        y1 = p1[t.header.frame_id].pose.position.y
        x2 = p2[t.header.frame_id].pose.position.x
        y2 = p2[t.header.frame_id].pose.position.y
        
        xb_s = t.beam[0].x  # start
        yb_s = t.beam[0].y
        xb_f = t.beam[1].x  # finish
        yb_f = t.beam[1].y

        #print x1, x2, xb_s, xb_f 
        #print y1, y2, yb_s, yb_f 

        # dealing with vertical beam -> yb_s == yb_f
        if yb_s >= y1 and yb_s < y2:
            # left to right
            if abs(y2-y1) < 1e-6:
                xp = x1
            else:
                xp = x1+(yb_s-y1)*(x2-x1)/(y2-y1)
            if '-n' in k and xp >= xb_s and xp <= xb_f:
                tv.append(t.pos)
            elif '-s' in k and xp >= xb_f and xp <= xb_s:
                tv.append(t.pos)
                
        elif yb_s >= y2 and yb_s < y1:
            # right to left
            if abs(y2-y1) < 1e-6:
                xp = x2
            else:
                xp = x2+(yb_s-y2)*(x1-x2)/(y1-y2)
            if '-n' in k and xp >= xb_s and xp <= xb_f:
                tv.append(t.neg)
            elif '-s' in k and xp >= xb_f and xp <= xb_s:
                tv.append(t.neg)

    # handle horizontal beams
    for i in range(len(h_sort)):
        (k, h) = h_sort.popitem(False)
        t = topos[k]
        xb_s = t.beam[0].x  # start
        yb_s = t.beam[0].y
        xb_f = t.beam[1].x  # finish
        yb_f = t.beam[1].y

        # dealing with horizontal beam -> xb_s == xb_f
        if xb_s >= x1 and xb_s < x2:
            # south to north
            if abs(x2-x1) < 1e-6:
                yp = y1
            else:
                yp = y1+(xb_s-x1)*(y2-y1)/(x2-x1)
            if '-e' in k and yp >= yb_s and yp <= yb_f:
                th.append(t.pos)
            elif '-w' in k and yp >= yb_f and yp <= yb_s:
                th.append(t.pos)
        elif xb_s >= x2 and xb_s < x1:
            # north to south
            if abs(x2-x1) < 1e-6:
                yp = y2
            else:
                yp = y2+(xb_s-x2)*(y1-y2)/(x1-x2)
            if '-e' in k and yp >= yb_s and yp <= yb_f:
                th.append(t.neg)
            elif '-w' in k and yp >= yb_f and yp <= yb_s:
                th.append(t.neg)
    
    if tv == []:
        tv = ['']
    if th == []:
        th = ['']
    return (tv, th)
