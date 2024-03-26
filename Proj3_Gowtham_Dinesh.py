
import numpy as np
import cv2
from queue import PriorityQueue
import math
import matplotlib.path as mplPath
import matplotlib.pyplot as plt

class PossibleNode :
    def __init__ (self, x , y, angle,  parent, g, h):
        """
        Represents a possible node in the A* algorithm.

        Parameters:
        - x (int): The x-coordinate of the node.
        - y (int): The y-coordinate of the node.
        - angle (int): The angle of the node.
        - parent (PossibleNode): The parent node.
        - g (float): The cost from the start node to this node.
        - h (float): The estimated cost from this node to the goal node.
        """
        self.x = x
        self.y = y
        self.angle = angle
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h

    def get_angle(self):
        """
        Returns the angle of the node.
        """
        return self.angle

    def get_points(self):
        """
        Returns the (x, y) coordinates of the node.
        """
        return (self.x , self.y)
    
    def get_total_cost(self):
        """
        Returns the total cost of the node (g + h).
        """
        return self.g + self.h
    
    def get_parent_node(self):
        """
        Returns the parent node.
        """
        return self.parent
    
    def __hash__ (self):
        """
        Returns the hash value of the node.
        """
        return hash((self.x, self.y))
    
    def __eq__(self, other):
        """
        Checks if two nodes are equal based on their coordinates.
        """
        return self.x == other.x and self.y == other.y 
    
    def __lt__(self, other):
        """
        Compares two nodes based on their total cost.
        """
        return self.f < other.f

# Global variables
y = 500
robotClearance = 0
robot_stepSize = 0
frame = np.zeros((500, 1200, 3), dtype=np.uint8 )
newColor = (220, 180, 120)
writer_video = cv2.VideoWriter('gow_dinesh.avi', cv2.VideoWriter_fourcc(*'XVID'), 1000, (1200,500))

def vertex_of_hex() :
    """
    Returns the vertices of a hexagon.

    Returns:
    - vertex_of_hex (list): List of (x, y) coordinates of the hexagon vertices.
    """
    vertex_of_hex = []
    for i in range(6):
        rad_angle = math.radians(30 + 60 * i)  
        x_cord = int(650 + 150 * math.cos(rad_angle))
        y_cord = int(250 + 150 * math.sin(rad_angle))
        vertex_of_hex.append((x_cord, y_cord))
    return vertex_of_hex

def inital_final_goals():
    """
    Prompts the user to enter the initial and goal points, along with orientations.

    Returns:
    - initial_point (tuple): The (x, y) coordinates of the initial point.
    - goal_point (tuple): The (x, y) coordinates of the goal point.
    - initial_orientation (int): The orientation of the robot at the initial point.
    - goal_orientation (int): The orientation of the robot at the goal point.
    """
    global robotClearance
    global robot_stepSize
    
    while True:
        x_coordinate, y_coordinate, initial_orientation = map(int, input("Enter the x and y coordinates of the initial point, followed by the orientation of the robot, separated by spaces: ").split())
        initial_point = (x_coordinate,y_coordinate)

        x_coordinate_goal, y_coordinate_goal, goal_orientation = map(int, input("Enter the x and y coordinates of the goal point, followed by the orientation of the robot at the goal point, separated by spaces: ").split()) 
        robotClearance = int(input("input robotClearance: "))
        robot_stepSize = int(input("step size is : "))
        goal_point = (x_coordinate_goal, y_coordinate_goal)

        if could_move(initial_point) and could_move(goal_point):
            return initial_point, goal_point , initial_orientation, goal_orientation

hex_pt = np.array(vertex_of_hex())
geometry_shape = np.array([[900, y-450], [1100, y-450], [1100, y-50], [900, y-50], [900, y-125], [1020, y-125], [1020, y-375], [900, y-375]])

cv2.polylines(frame, [hex_pt], True, (255, 0, 255), 5)
cv2.fillPoly(frame, [hex_pt], (0, 255, 255))

cv2.rectangle(frame, (100, y-500), (175, y-100), (255, 0, 255), 5)
cv2.rectangle(frame, (100, y-500), (175, y-100), (0, 255, 255), -1)

cv2.rectangle(frame, (275, y-400), (350, y), (255, 0, 255), 5)
cv2.rectangle(frame, (275, y-400), (350, y), (0, 255, 255), -1)

cv2.polylines(frame, [geometry_shape], True, (255, 0, 255), 5)
cv2.fillPoly(frame, [geometry_shape],(0, 255, 255))

def could_move(cord):
    """
    Checks if a given coordinate is a valid move.

    Parameters:
    - cord (tuple): The (x, y) coordinates to check.

    Returns:
    - bool: True if the coordinate is a valid move, False otherwise.
    """
    if cord[0] < 6 or cord[0] > 1195 or cord[1] < 6 or cord[1] > 495:
        return False 
    cord_color = frame[cord[1], cord[0]]
    if cord_color[0] == 0 and cord_color[1] == 0 and cord_color[2] == 0:
        return True
    if cord_color[0] == 220 and cord_color[1] == 180 and cord_color[2] == 120:
        return True
    return False

def distance(pt1, pt2):
    """
    Calculates the Euclidean distance between two points.

    Parameters:
    - pt1 (tuple): The (x, y) coordinates of the first point.
    - pt2 (tuple): The (x, y) coordinates of the second point.

    Returns:
    - float: The Euclidean distance between the two points.
    """
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

def move(node, step, angle):
    """
    Moves the robot to a new node based on the given step size and angle.

    Parameters:
    - node (PossibleNode): The current node.
    - step (int): The step size for the movement.
    - angle (int): The angle for the movement.

    Returns:
    - PossibleNode: The new node after the movement, or None if the movement is not possible.
    """
    new_angle = node.get_angle() + angle
    new_coordinates = (node.x + int(round((math.cos(math.radians(new_angle))*step ))), node.y + int(round((math.sin(math.radians(new_angle))*step))))
    robotClearance_check = (node.x + int(round((math.cos(math.radians(new_angle))*robotClearance))), node.y + int(round((math.sin(math.radians(new_angle))*robotClearance))))
    if not could_move(new_coordinates) or not could_move(robotClearance_check):
        return None
    return PossibleNode(new_coordinates[0], new_coordinates[1], new_angle, parent=node, g=node.g + step, h=distance(new_coordinates, end_point))

def backtracking(node):
    """
    Performs backtracking from the goal node to the start node and visualizes the path.

    Parameters:
    - node (PossibleNode): The goal node.
    """
    backtrackPath = []
    while node is not None:
        backtrackPath.append((node.x , node.y))
        frame[node.y, node.x] = (173, 46, 0)
        node = node.get_parent_node()

    for i in range(len(backtrackPath) - 1):
        cv2.arrowedLine(frame, backtrackPath[i], backtrackPath[i + 1], (173, 46, 0), 1 , tipLength= 0.25)
        writer_video.write(frame)

if __name__ == "__main__":
    start_point, end_point, angle, goal_angle = inital_final_goals()
    a_star_implementation(start_point, end_point, angle)
    writer_video.release()
    plt.imshow(frame)
    plt.gca().invert_yaxis()
    plt.show()
