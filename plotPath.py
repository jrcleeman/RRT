import matplotlib.pyplot as plt
import matplotlib.patches as patches

class Plotting:
    def __init__(self, startNodeArray, goalNodeArray, boundary, circleObstacles, rectangleObstacles):
        #Setting initial and goal arrays
        self.startNodeArray = startNodeArray
        self.goalNodeArray = goalNodeArray
        self.boundary = boundary
        self.circleObstacles = circleObstacles
        self.rectangleObstacles = rectangleObstacles

    #called when rrt finished
    def animation(self, nodelist, path, animation):
        #Plot Skeleton
        self.plot_grid("RRT")
        #Plot all nodes explored
        self.plot_visited(nodelist, animation)
        #Plot proper path
        self.plot_path(path)

    #Plots Skeleton
    def plot_grid(self, name):
        fig, ax = plt.subplots()
        #plot boundary
        for (ox, oy, w, h) in self.boundary:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        #plot rectangle obs
        for (ox, oy, w, h) in self.rectangleObstacles:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        #plot circle obs
        for (ox, oy, r) in self.circleObstacles:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        #plot start and end nodes
        plt.plot(self.startNodeArray[0], self.startNodeArray[1], "bs", linewidth=3)
        plt.plot(self.goalNodeArray[0], self.goalNodeArray[1], "gs", linewidth=3)
        
        #plot name
        plt.title(name)
        #plot sizing
        plt.axis("equal")

    #plot all paths explored
    @staticmethod
    def plot_visited(nodelist, animation):
        #only here for RRT
        if animation:
            count = 0
            #for all nodes explored
            for node in nodelist:
                count += 1
                if node.parent:
                    #plot line between 2 points
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    
                    #pause plotting so user can see progress
                    if count % 10 == 0:
                        plt.pause(0.001)
        
        #doesnt do fancy animation
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    #Plot path from start to goal
    @staticmethod
    def plot_path(path):
        #make sure path is valid
        if len(path) != 0:
            #plot all coordinates in path array
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            #basic pause
            plt.pause(0.01)
        plt.show()
