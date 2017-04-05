
import rospy, tf, numpy, math, random, Queue
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

class cell: #stores the probability of the cell being occupied and its f(n) cost
            #also stores the position (assuming origin is at the top left corner)
    prob = 50 #start at 50/50

    cost = -1 #unexplored (cost unknown) f(n)
    g = 0 #g(n)
    h = 0 #h(n)

    x = 0 #x position
    y = 0 #y position

    def __init__(self, probability, xLoc, yLoc):
        prob = probability
        x = xLoc
        y = yLoc

def heuristic(current,goal): #returns h(n) euclidian distance to goal
    math.sqrt(((goal.x-current.x)**2) + ((goal.y-current.y)**2)) #distance formula
    
def getMap(msg): #callBack for the map topic
    grid = get2DArray(occGrid.data, msg.info.width, msg.info.height)#get a 2D array version of the grid
    return grid

def aStar(grid, start, goal): #takes a grid (2D array of cell objects), start and goal (both cells) 
    openSet = Queue.PriorityQueue(maxsize=0) #create a set to store all discovered nodes yet to be explored
    closedSet = [] #just a list because all we care about is whether or not something is here already
    
    start.g = 0 #by definition
    start.h = heuristic(start, goal)#euclidian distance to goal
    start.cost = start.h
    openSet.put((start.cost,start)) #add start to the queue
    
    while ( not openSet.empty() ):#for as long as unexpanded (but discovered) nodes exist
        
        #update the sets:
        current = openSet.get() #gets the lowest cost node (based on f(n))
        closedSet.append(current) #put the current node in the set of closed nodes

        #are we there yet?!
        if (current == goal):
            #yep!
            return #!!!figure out return data!!!

        #get all the children:
        x = current.x
        y = current.y
        
        #its not hugely important, but this is how I'm numbering the childern:
        #  7 0 1
        #  6 C 2
        #  5 4 3

        children = []#create a list of the childern

        try: #the try except is needed because this method will try to call for cells that don't exist
             #this will catch the list index out of range error and ignore it (and skip that non-exitent child)
            children.append(grid[x][y-1]) #y-1 because origin is at top left
        except:
            pass
        try: #child 1
            children.append(grid[x+1][y-1])
        except:
            pass
        try: #child 2
            children.append(grid[x+1][y])
        except:
            pass
        try: #child 3
            children.append(grid[x+1][y+1])
        except:
            pass
        try: #child 4
            children.append(grid[x][y+1])
        except:
            pass
        try: #child 5
            children.append(grid[x-1][y-1])
        except:
            pass
        try: #child 6
            children.append(grid[x-1][y])
        except:
            pass
        try: #child 7
            children.append(grid[x-1][y-1])
        except:
            pass
        
        #go through all of the children/neighbors:
        for child in children: #make sure to go through everything

            if child not in closedSet: #if the cell isn't already expanded (if it is we ignore it)

                if (child.prob < 50): #probably not an obstacle (if its an obstacle we ignore it)
                    child.g = current.g + 1 #may want to change this to dist fomula later!!!
                    child.h = heuristic(child,goal)
                    child.cost = child.g + child.h #total cost
                
                    openSet.put((child.cost,child)) #add to priority queue

        #END MAIN WHILE
    print("no solutions exist")
    #END


def get2DArray(data, width, height): #an absolutely thrilling function to take a 1D array and break
                                     #it into a 2D array (a grid) given a width and height

    grid = [[0 for row in range(0, width)] for col in range(0, height)] #create the 2D array
    
    #create index variable
    i = 0 #index for outer loop (keeps track of the row)
    j = 0 #index for inner loop (keeps track of the column)
    k = 0 #index for data (never gets reset)

    while (i < height): #go through all rows (starting at the top (0,0))
        j = 0 #reset index (start at the start of the new row)

        while (j < width): #go through a single row
            grid[j][i] = cell(data[k], i, j)#creates a cell object
            j+=1
            k+=1

        i+=1
    
    return grid

# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('aStar')

    map_sub = rospy.Subscriber('/map', OccupancyGrid, getMap, queue_size=1) #get the occupancy grid
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting A*"
    
    while(1):
        pass
    
