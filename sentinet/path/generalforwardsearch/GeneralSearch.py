from random import randint
import operator

width_ = 10
height_ = 10

"""
A Generic forwards search algorithm

A State space needs:
    1. A non empty state space X with countable states (our map)
    2. For each state in X, a finite action space U(x)
    3. A State transition function f(u, x) existing in X (x' = f(u, x))
    4. Initial state (x1)
    5. A Goal set of states that exist in X
"""

class Map:
    def __init__(self, starting, width = width_, height = height_):
        self.map = [[0 for x in range(width)] for y in range(height)]
        self.height = height
        self.width = width
        self.visited = [starting]
        self.transitions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    # More mathematically properly, not optimized
    def GenerateTransitionSet(self, pos):
        U = []
        for i in self.transitions:
            trans = tuple(map(operator.add, pos, i))
            if trans in self.map and trans not in self.visited:
                U.append(i)
        return U

    # Transition function appends to visited and operates on u, x
    def transition(self, x, u):
        xprime = tuple(map(operator.add, pos, i))
        self.visited.append(xprime)
        return xprime



# Define our starting points
x1 = (randint(0, height_), randint(0, width_))
X = Map(x1)

# Define our goal state set
num_goal_states = randint(0, height_ * width_) # Number of goal states in X
Xg = []

for i in range(num_goal_states):
    x = randint(0, height_)
    y = randint(0, width_)
    while (x, y) in Xg and x != x1[0] and y != x1[1]:
        x = randint(0, height_)
        y = randint(0, width_)
    Xg.append((x, y))

Unvisted = []
Unvisted.append(x1)

print(x1, '\n\n')
print(X.map, '\n\n')
print(Xg, '\n\n')
# Transition functions
while len(Unvisted) != 0:
    print(Unvisted)
    pos = Unvisted[0]
    if pos in Xg:
        print(pos)
        break
    
    U = X.GenerateTransitionSet(pos)
    print(U)
    for i in U:
        xprime = X.transition(pos, i)
        Unvisted.append(xprime)
    print(pos)
    Unvisted.remove(pos)
