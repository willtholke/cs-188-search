# search.py
# ---------
#New comment
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from ctypes import resize
import queue
import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm. Avoids expanding
    any already visited states. Stack implements LIFO.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    stack, s = util.Stack(), problem.getStartState()
    visited, prev, prev[s] = [], {}, []
    stack.push(s)
    while (not stack.isEmpty()):
        node = stack.pop()
        if problem.isGoalState(node):
            return prev[node]
        if node not in visited:
            visited.append(node)
            for x in problem.getSuccessors(node):
                successor, action = x[0], x[1]
                prev[successor] = prev[node].copy()
                prev[successor].append(action)
                stack.push(successor)
    return res


def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    queue = util.Queue()
    node = problem.getStartState()
    queue.push([node, None, None])
    visited = set()
    prev = {}
    prev[node] = []
    min = None
    res = []
    while (not queue.isEmpty()):
        tup = queue.pop()
        node = tup[0]
        pare = tup[1]
        dire = tup[2]
        if node in visited:
            continue
        if pare != None:
            prev[node] = prev[pare].copy()
            prev[node].append(dire)
        visited.add(node)
        if problem.isGoalState(node):
            return prev[node]
        for x in problem.getSuccessors(node):
            queue.push([x[0], node, x[1]])
    return res
def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    queue = util.PriorityQueue()
    node = problem.getStartState()
    queue.push(node, 0)
    visited = []
    prev = {}
    priority = {}
    prev[node] = []
    priority[node] = 0
    
    while (not queue.isEmpty()):
        if (queue.isEmpty()):
            return util.raiseNotDefined
        node = queue.pop()
        if node in visited:
            continue
        visited.append(node)
        if problem.isGoalState(node):
            return prev[node]
        for x in problem.getSuccessors(node):
            if x[0] in priority:
                if priority[x[0]] > priority[node] + x[2]:
                    priority[x[0]] = priority[node] + x[2]
                    queue.update(x[0], priority[x[0]])
                    prev[x[0]] = prev[node].copy()
                    prev[x[0]].append(x[1])
            else: 
                priority[x[0]] = priority[node] + x[2]
                queue.push(x[0], priority[x[0]])
                prev[x[0]] = prev[node].copy()
                prev[x[0]].append(x[1])


        #visited.append(node)
    # while (not queue.isEmpty()):
    #     node = queue.pop()
    #     visited.append(node)
    #     if problem.isGoalState(node):
    #         return prev[(node, priority[node])]
    #         #Once something is in queue
    #         #Update weights when looking through successors add backwards cost
    #         #Then add to visited
    #     # for i in problem.getSuccessors(node):
    #     #     if i[0] in visited:
    #     #         queue.update(i[0],priority[node] + i[2])
    #     for x in problem.getSuccessors(node):
    #         tup = (x[0], x[2] + priority[node])
    #         if tup in visited:
    #             #queue.update(x[0], priority[node] + x[2])
    #             continue
    #         visited.append(tup)
    #         priority[x[0]] = x[2] + priority[node]
    #         prev[(x[0], priority[x[0]])] = prev[(node, priority[node])].copy()
    #         #priority[x[0]] = priority[node] + x[2]
    #         prev[(x[0], priority[x[0]])].append(x[1])
    #         queue.update(x[0], priority[x[0]])
    return res

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    queue = util.PriorityQueue()
    node = problem.getStartState()
    queue.push([node, None, None, 0], 0)
    visited = set()
    prev = {}
    prev[node] = []
    res = []
    priority = {}
    priority[node] = 0
    while (not queue.isEmpty()):
        tup = queue.pop()
        node = tup[0]
        pare = tup[1]
        dire = tup[2]
        cost = tup[3]
        if node in visited:
            continue
        if pare != None:
            prev[node] = prev[pare].copy()
            prev[node].append(dire)
            priority[node] = priority[pare] + cost
        visited.add(node)
        if problem.isGoalState(node):
            return prev[node]
        for x in problem.getSuccessors(node):
            queue.push([x[0], node, x[1], x[2]], priority[node] + x[2] + heuristic(x[0], problem))
    return None

class Node:
    def __init__(self):
        self.true_cost = 0
        self.parent = None

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
