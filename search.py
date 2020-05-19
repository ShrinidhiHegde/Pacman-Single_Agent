# search.py
# ---------
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

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    # Stack to hold the frontier in DFS
    frontier = util.Stack()
    # frontier contains state and the path to that state.
    frontier.push((problem.getStartState(), []))
    visitedSet = [problem.getStartState()]
    while not frontier.isEmpty():
        # We get the path along with state, as the path to corresponding nodes are stored in frontier.
        (currentState, solution) = frontier.pop()
        visitedSet.append(currentState)
        if problem.isGoalState(currentState):
            return solution
        successors = problem.getSuccessors(currentState)
        for i in successors:
            if problem.isGoalState(currentState):
                return solution
            # push successors to frontier if it is not in visited set
            if i[0] not in visitedSet:
                frontier.push((i[0], solution + [i[1]]))
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # Queue holds the frontier nodes of BFS
    frontier = util.Queue()
    frontier.push((problem.getStartState(), []))
    visitedSet = [problem.getStartState()]
    while not frontier.isEmpty():
        # We get the path along with state, as the path to corresponding nodes are stored in frontier.
        (currentState, solution) = frontier.pop()
        if problem.isGoalState(currentState):
            return solution
        successors = problem.getSuccessors(currentState)
        for i in successors:
            # push successors to frontier if it is not in visited set
            if i[0] not in visitedSet:
                frontier.push((i[0], solution + [i[1]]))
                visitedSet.append(i[0])
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    frontier = util.PriorityQueue()
    cost = 0
    # frontier stores even the cost along with path and state.
    frontier.push((problem.getStartState(), [], cost), cost)
    visitedSet = [(problem.getStartState(), 0)]
    while not frontier.isEmpty():
        (currentState, solution, cost) = frontier.pop()
        if problem.isGoalState(currentState):
            return solution
        successors = problem.getSuccessors(currentState)
        for i in successors:
            if i[0] not in [visitedState[0] for visitedState in visitedSet]:
                frontier.push((i[0], solution + [i[1]], cost + i[2]), cost + i[2])
                visitedSet.append((i[0], cost + i[2]))
            # Cost of the visited nodes are also updated.
            else:
                visitedCost = 0
                for j in visitedSet:
                    if j[0] == i[0]:
                        visitedCost = j[1]
                        break
                if cost + i[2] < visitedCost:
                    frontier.update((i[0], solution + [i[1]], cost + i[2]), cost + i[2])
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    frontier = util.PriorityQueue()
    cost = 0
    frontier.push((problem.getStartState(), [], cost),
                  cost + heuristic(problem.getStartState(), problem))
    # cost consists of heuristic function as well.
    visitedSet = [(problem.getStartState(), 0 + heuristic(problem.getStartState(), problem))]
    while not frontier.isEmpty():
        (currentState, solution, cost) = frontier.pop()
        if problem.isGoalState(currentState):
            return solution
        successors = problem.getSuccessors(currentState)
        for i in successors:
            visitedSate = [visitedState[0] for visitedState in visitedSet]
            if i[0] not in visitedSate:
                updatedCost = cost + i[2] + heuristic(i[0], problem)
                frontier.push((i[0], solution + [i[1]], cost + i[2]), updatedCost)
                visitedSet.append((i[0], updatedCost))
            elif i[0] in visitedSate:
                index = visitedSate.index(i[0])
                visitedCost = [visitedCost[1] for visitedCost in visitedSet]
                updatedCost = cost + i[2] + heuristic(i[0], problem)
                if (cost + i[2] + heuristic(i[0], problem)) < visitedCost[index]:
                    frontier.update((i[0], solution + [i[1]], cost + i[2]), updatedCost)
                    visitedSet.pop(index)
                    visitedSet.append((i[0], updatedCost))

    util.raiseNotDefined()


def getNodeFromState(state):
    if len(state) > 1:
        if type(state[0]) is tuple:
            return state[0]
        else:
            return state
    else:
        return state

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
