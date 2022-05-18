# A* Path Finding Algorithm for 2D Grid World
## AIM

To develop a code to find the route from the source to the destination point using A* algorithm for 2D grid world.

## THEORY
We try to use the A* algorithm to navigate through a 2D Gird environment. We provide the algorithm with the inital and goal states, and then let the algorithm calculate the Heuristic function to decide the path nodes. And finally, we return the path nodes to the user. 

## DESIGN STEPS

### STEP 1:
Build a 2D grid world with initial state and goal state
<br>Initial State: (2,2)
<br>Goal State: (5,8)
### STEP 2:
Mention the Obstacles in the 2D grid World
### STEP 3:
Define the function for the distance function for the heuristic function
### STEP 4:
Pass all the values to the GirdProblem, and print the solution path.

## Draw the 2D 

![WhatsApp Image 2022-05-17 at 9 10 52 PM (1)](https://user-images.githubusercontent.com/75264748/168864493-c9f15f15-7469-4062-924f-9446ca628609.jpeg)



## PROGRAM
```
/*
Developed By:KRISHNA MOORTHY S
Reg. No: 212220230025
*/
```
```python

%matplotlib inline
import matplotlib.pyplot as plt
import random
import math
import sys
from collections import defaultdict, deque, Counter
from itertools import combinations
import heapq

class Problem(object):
    """The abstract class for a formal problem. A new domain subclasses this,
    overriding `actions` and `results`, and perhaps other methods.
    The default heuristic is 0 and the default action cost is 1 for all states.
    When yiou create an instance of a subclass, specify `initial`, and `goal` states 
    (or give an `is_goal` method) and perhaps other keyword args for the subclass."""

    def __init__(self, initial=None, goal=None, **kwds): 
        self.__dict__.update(initial=initial, goal=goal, **kwds) 
        
    def actions(self, state):        
        raise NotImplementedError
    def result(self, state, action): 
        raise NotImplementedError
    def is_goal(self, state):        
        return state == self.goal
    def action_cost(self, s, a, s1): 
        return 1
    
    def __str__(self):
        return '{0}({1}, {2})'.format(
            type(self).__name__, self.initial, self.goal)
            
class Node:
    "A Node in a search tree."
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __str__(self): 
        return '<{0}>'.format(self.state)
    def __len__(self): 
        return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): 
        return self.path_cost < other.path_cost
        
failure = Node('failure', path_cost=math.inf) 
cutoff  = Node('cutoff',  path_cost=math.inf)


def expand(problem, node):
    "Expand a node, generating the children nodes."
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)
        

def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []  
    return path_actions(node.parent) + [node.action]


def path_states(node):
    "The sequence of states to get to this node."
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]
    
    
class PriorityQueue:
    """A queue in which the item with minimum f(item) is always popped first."""

    def __init__(self, items=(), key=lambda x: x): 
        self.key = key
        self.items = [] # a heap of (score, item) pairs
        for item in items:
            self.add(item)
         
    def add(self, item):
        """Add item to the queuez."""
        pair = (self.key(item), item)
        heapq.heappush(self.items, pair)

    def pop(self):
        """Pop and return the item with min f(item) value."""
        return heapq.heappop(self.items)[1]
    
    def top(self): return self.items[0][1]

    def __len__(self): return len(self.items)
    
   
 def best_first_search(problem, f):
    "Search nodes with minimum f(node) value first."
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem, node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
    return failure

def g(n): 
    return n.path_cost
    
    
    
class GridProblem(Problem):
    """Finding a path on a 2D grid with obstacles. Obstacles are (x, y) cells."""

    def __init__(self, initial=(15, 30), goal=(130, 30), obstacles=(), **kwds):
        Problem.__init__(self, initial=initial, goal=goal, 
                         obstacles=set(obstacles) - {initial, goal}, **kwds)

    directions = [(-1, -1), (0, -1), (1, -1),
                  (-1, 0),           (1,  0),
                  (-1, +1), (0, +1), (1, +1)]
    
    def action_cost(self, s, action, s1): 
        return straight_line_distance(s, s1)
    
    def h(self, node): 
        return straight_line_distance(node.state, self.goal)
                  
    def result(self, state, action): 
        "Both states and actions are represented by (x, y) pairs."
        return action if action not in self.obstacles else state
    
    def actions(self, state):
        """You can move one cell in any of `directions` to a non-obstacle cell."""
        
        
        x,y = state
        return {(x+dx,y+dy) for(dx,dy) in self.directions}-self.obstacles
        # Write your code here
        return {}
        
        
def straight_line_distance(A, B):
    "Straight-line distance between two points."
    # Write your code here
    return sum(abs(a-b)**2 for(a,b) in zip(A,B))**0.5
    


def g(n): 
    return n.path_cost
    
    

def astar_search(problem, h=None):
    """Search nodes with minimum f(n) = g(n) + h(n)."""
    h = h or problem.h
    return best_first_search(problem, f=lambda n: g(n) + h(n))
obstacles={(0,3),(0,4),(0,5),(1,4),(2,4),(2,1),(3,1),(3,2),(0,5),(0,7),(0,9),(1,9),(0,9),(2,9),(3,9),(4,9),(2,7),(2,8),(3,6),(3,7),(3,8)}

grid1 = GridProblem(initial=(1,1), goal =(4,8) ,obstacles=obstacles)
s1 = astar_search(grid1)


path_states(s1)
```


## OUTPUT:
![ai exp4](https://user-images.githubusercontent.com/75264748/168864033-c569a27b-c0d0-46e5-97ac-44b891c5048f.jpg)
![ai  exp4](https://user-images.githubusercontent.com/75264748/168864057-f2b84ffb-86cc-46cd-81e3-5cf5242e8a50.jpg)



The algorithm is able to find the solution path for the given problem. But the solution path, might not be the shortest path to reach the goal state.

## RESULT:
Hence, A* Algorithm was implemented to find the route from the source to the destination point in a 2D gird World.
