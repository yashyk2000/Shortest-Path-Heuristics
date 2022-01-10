# -*- coding: utf-8 -*-
"""
Created on Thu Oct  7 10:00:50 2021

@author: YASH
"""


#importing Libraries
import numpy as np
import pandas as pd
import time
from datetime import timedelta

states = input().split(' ')
try:    
    initial,goal = states 
    initial = initial.upper()
    goal = goal.upper()
    print("INITIAL STATE:",initial)
    print("GOAL STATE:",goal)
    print()

    driving = pd.read_csv('drivingcsv-141847.csv')
    straight_line = pd.read_csv('straightlinecsv-141848.csv')

    arr = driving.to_numpy()[:,0]

    connect_graph = driving.to_numpy()[:,1:] #actual distance
    heuristic_graph = straight_line.to_numpy()[:-3,1:-2] # Air/Flight distance


    def shortest_path_using_Astar(connect_graph,heuristic_graph,start,goal):
        import heapq

        # initialization of variables
        path = []  # shortest path
        explored_nodes = set() #closed List
        path_list = [] #open List
        start_path = []
        start_path.append(start)
        start_path_cost = 0
        start_est_distance = heuristic_graph[start][goal]
        start_total_cost = start_path_cost + start_est_distance
        
        start_record = [start_total_cost, start_path_cost, start_est_distance , start_path]

        heapq.heappush(path_list, start_record) 

        while len(path_list)>0:
            # use a heap structure to pop out the path with the shortest total cost so far
            #print(path_list)
            current_record = heapq.heappop(path_list)
            #print(current_record)
            current_path = current_record[3]
            current_path_cost = current_record[1]
            current_node = current_record[3][-1]
            explored_nodes.add(current_node)
            
            # goal test
            if current_node == goal:
                path = current_path
                break
            
            # explore all of the new nodes that is connected with the current node
            for new_node in range(len(connect_graph[current_node])):
                # ignore the ones already explored
                if new_node not in explored_nodes and connect_graph[current_node][new_node]!=-1:
                    # generate new paths for next round of exploration
                    new_path = current_path + [new_node]
                    new_path_cost = current_path_cost+heuristic_graph[current_node][new_node]
                    new_est_distance = heuristic_graph[new_node][goal]
                    new_total_cost = new_path_cost + new_est_distance
                    new_record = [new_total_cost, new_path_cost, new_est_distance, new_path]
                    heapq.heappush(path_list, new_record)         
                    
        return path,current_path_cost


    def shortest_path_using_Greedy_BFS(connect_graph,heuristic_graph,start,goal):
        
        
        import heapq
        
        # initialization of variables
        path = []  # shortest path
        explored_nodes = set()
        path_list = []
        start_path = []
        start_path.append(start)
        start_path_cost = 0
        start_est_distance = heuristic_graph[start][goal]
        start_total_cost = start_est_distance
        
        start_record = [start_total_cost, start_path_cost, start_est_distance , start_path]
        
        heapq.heappush(path_list, start_record)
        
        while len(path_list)>0:
            # use a heap structure to pop out the path with the shortest total cost so far
            #print(path_list)
            current_record = heapq.heappop(path_list)
            #print(current_record)
            current_path = current_record[3]
            current_path_cost = current_record[1]
            current_node = current_record[3][-1]
            explored_nodes.add(current_node)
            
            # goal test
            if current_node == goal:
                path = current_path
                break
            
            # explore all of the new nodes that is connected with the current node
            for new_node in range(len(connect_graph[current_node])):
                # ignore the ones already explored
                if new_node not in explored_nodes and connect_graph[current_node][new_node]!=-1:
                    # generate new paths for next round of exploration
                    new_path = current_path + [new_node]
                    new_path_cost = current_path_cost+heuristic_graph[current_node][new_node]
                    new_est_distance = heuristic_graph[new_node][goal]
                    new_total_cost = new_est_distance
                    new_record = [new_total_cost, new_path_cost, new_est_distance, new_path]
                    heapq.heappush(path_list, new_record)
                    
                    
        return path,current_path_cost



    start = np.where(arr == initial)[0][0]
    goal = np.where(arr == goal)[0][0]


    #printing path using Greedy Best First search
    start_time = time.time()
    path,path_cost = shortest_path_using_Greedy_BFS(connect_graph,heuristic_graph, start,goal )
    ex_time = timedelta(seconds= time.time() - start_time).total_seconds()
    print()
    print("Greedy Best First Search:")
    print("Solution path:",end=" ")
    for i in path:
        print(arr[i],end=' ')
    print()
    print("Number of states on a path:",len(path))
    print("Path cost:",path_cost)
    print("Execution time:",ex_time," seconds")
    print()


    #printing path using A* Search
    start_time = time.time()
    path,path_cost= shortest_path_using_Astar(connect_graph,heuristic_graph, start,goal )
    ex_time = timedelta(seconds=time.time() - start_time).total_seconds()
    print("A* Search:")
    print("Solution path:",end=" ")
    for i in path:
        print(arr[i],end=' ')
    print()
    print("Number of states on a path:",len(path))
    print("Path cost:",path_cost)
    print("Execution time:",ex_time," seconds") 
except:
    print("ERROR: Not enough or too many input arguments.")        
    