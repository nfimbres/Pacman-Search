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
    
    stack = util.Stack()
    visited = []
    action_list = []
    stack.push((problem.getStartState(), action_list))
    
    while stack:
        
        node, actions = stack.pop()
        
        if not node in visited:
            
            visited.append(node)
            
            if problem.isGoalState(node):
                return actions
            
            for s in problem.getSuccessors(node):
                coordinate, direction, cost = s
                nextActions = actions + [direction]
                stack.push((coordinate, nextActions))
                
    return []
    
    
def breadthFirstSearch(problem):
    stack = util.Queue()
    visited = []
    action_list = []
    stack.push((problem.getStartState(), action_list))
    
    while stack:
        
        node, actions = stack.pop()
        
        if not node in visited:
            
            visited.append(node)
            
            if problem.isGoalState(node):
                return actions
            
            for s in problem.getSuccessors(node):
                coordinate, direction, cost = s
                nextActions = actions + [direction]
                stack.push((coordinate, nextActions))
                
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
    
    queue = util.PriorityQueue()
    visited = []
    action_list = []
    queue.push((problem.getStartState(), action_list), heuristic(problem.getStartState(), problem))
    
    while queue:
        
        node, actions = queue.pop()
        
        if not node in visited:
            
            visited.append(node)
            
            if problem.isGoalState(node):
                return actions
            
            for successor in problem.getSuccessors(node):
                
                coordinate, direction, cost = successor
                nextActions = actions + [direction]
                nextCost = problem.getCostOfActions(nextActions) + heuristic(coordinate, problem)
                queue.push((coordinate, nextActions), nextCost)
                
    return []
    
    
    
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
