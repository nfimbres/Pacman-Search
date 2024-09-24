# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    stack = util.Stack() #Initialize a stack to manage nodes in LIFO order for DFS
    visited = [] #List to track visited nodes to prevent cycles
    action_list = [] #List to store the sequence of actions to reach a goal
    stack.push((problem.getStartState(), action_list)) #Push the start state along with the initial action list to the stack
    
    #Continue while there are nodes to explore in the stack
    while stack:
        
        #Pop the top node from the stack (LIFO)
        node, actions = stack.pop()
        
        #If the node has not been visited yet
        if node not in visited:
            
            #Mark the node as visited
            visited.append(node)
            
            #If the node is the goal state, return the list of actions
            if problem.isGoalState(node):
                return actions
            
            #Expand the node by getting its successors
            for s in problem.getSuccessors(node):
                coordinate, direction, cost = s #s contains (state, direction, cost)
                nextActions = actions + [direction] #Create a new action list by adding the current action (direction)
                stack.push((coordinate, nextActions)) #Push the successor node and its actions to the stack
    
    #Return an empty list if no solution is found
    return []



def breadthFirstSearch(problem):
    queue = util.Queue() #Initialize a queue to manage nodes in FIFO order for BFS
    visited = [] #List to track visited nodes to prevent revisiting
    action_list = [] #List to store the sequence of actions to reach a goal
    queue.push((problem.getStartState(), action_list)) #Push the start state along with the initial action list to the queue
    
    #Continue while there are nodes to explore in the queue
    while queue:
        
        #Pop the front node from the queue (FIFO)
        node, actions = queue.pop()
        
        #If the node has not been visited yet
        if node not in visited:
            
            #Mark the node as visited
            visited.append(node)
            
            #If the node is the goal state, return the list of actions
            if problem.isGoalState(node):
                return actions
            
            # Expand the node by getting its successors
            for s in problem.getSuccessors(node):
                coordinate, direction, cost = s #s contains (state, direction, cost)
                nextActions = actions + [direction] #Create a new action list by adding the current action (direction)
                queue.push((coordinate, nextActions)) #Push the successor node and its actions to the queue
    
    #Return an empty list if no solution is found
    return []

def uniformCostSearch(problem):
    """
    Search the node of least total cost first using Uniform Cost Search (UCS).
    """
    startingNode = problem.getStartState() #Get the starting node (initial state)

    visited = set() #Initialize a set to keep track of visited nodes (already explored states)

    action_list = [] #Initialize an empty action list to track actions taken to reach a node

    queue = util.PriorityQueue()  #Initialize a priority queue to prioritize nodes with the least total cost

    queue.push((startingNode, action_list, 0), 0) #Push the starting node into the queue with its current cost set to 0
    #Each item in the queue is a tuple (node, actions, cost), and the priority is the cost

    while queue:  #Continue searching while there are nodes in the queue
        node, actions, prevCost = queue.pop() #Pop the node with the lowest cost from the priority queue

        if node not in visited: #If the node has not been visited, mark it as visited
            visited.add(node) #Add to visited set

            if problem.isGoalState(node): #If the node is the goal state, return the list of actions leading to this node
                return actions

            for nextNode, action, cost in problem.getSuccessors(node): #Explore all successors (children) of the current node
                newAction = actions + [action] #Create a new action list by appending the current action
                priority = prevCost + cost #Total cost to reach the next node
                queue.push((nextNode, newAction, priority), priority) #Push the successor node into the queue with its new cost as the priority

    util.raiseNotDefined() #If the goal state is not found, raise an error (this should not be reached)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    queue = util.PriorityQueue() #Initialize a priority queue for A* search
    visited = [] #List to track visited nodes to prevent revisiting
    action_list = [] #List to store the sequence of actions to reach a goal
    
    #Push the start state along with the initial action list and heuristic value to the queue
    queue.push((problem.getStartState(), action_list), heuristic(problem.getStartState(), problem))
    
    #Continue while there are nodes to explore in the priority queue
    while queue:
        node, actions = queue.pop() #Pop the node with the lowest priority (cost + heuristic)
        
        if node not in visited: #If the node has not been visited yet
            visited.append(node) #Mark the node as visited
            
            if problem.isGoalState(node): #If the node is the goal state, return the list of actions
                return actions
            
            for successor in problem.getSuccessors(node): #Expand the node by getting its successors
                
                coordinate, direction, cost = successor #successor contains (state, direction, cost)
                nextActions = actions + [direction] #Create a new action list by adding the current action (direction)
                nextCost = problem.getCostOfActions(nextActions) + heuristic(coordinate, problem) #Calculate total cost (path cost + heuristic)
                
                #Push the successor node, its actions, and the updated cost to the priority queue
                queue.push((coordinate, nextActions), nextCost)
    
    #Return an empty list if no solution is found
    return []

    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
