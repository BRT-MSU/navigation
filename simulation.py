import time
import myAlgorithm as alg
from tkinter import *

class simulation:
    def __init__ (self, width=800, height=600, spacing=50):
        self.width           = width
        self.height          = height
        self.spacing         = spacing
        self.obstacleMap     = []
        self.navigationPoints = [(0,0),(0,1)]

    def openSimulation(self):
        self.root = Tk()
        self.optionFrame = Frame(self.root)

        self.C = Canvas(self.root, bg="tan", width=self.width, height=self.height)
        self.index = self.canvas_gridSetup()  #Used in canvas_removeObject() to not delete grid lines
        self.C.bind("<Button-1>",self.canvas_leftClick)
        self.C.bind("<Button-3>",self.canvas_rightClick)

        self.var = IntVar()
        self.R1 = Radiobutton(self.optionFrame, text="Obstacle",  variable=self.var, value=1, command=self.option_sel)
        self.R2 = Radiobutton(self.optionFrame, text="Start/End", variable=self.var, value=2, command=self.option_sel)
        self.R1.grid(row=0, column=0, sticky=W)
        self.R2.grid(row=1, column=0, sticky=W)
        self.startButton = Button(self.optionFrame, text="Start")
        self.startButton.bind("<Button-1>", self.startSimulation)
        self.startButton.grid(row=2, column=0, sticky=W)

        self.C.grid(row=0, column=0, sticky=W)
        self.optionFrame.grid(row=0, column=1, sticky=W)

        self.root.mainloop()

    def canvas_gridSetup(self):
        r = int(self.height/self.spacing)
        c = int(self.width/self.spacing)
        i = 0
        for row in range(1,r):  #Create horizontal grid lines
            coord = 0, (row*self.spacing), self.width, (row*self.spacing)
            self.C.create_line(coord, fill="gray", tag="grid")
            i += 1
        for col in range(1,c):  #Create vertical grid lines
            coord = (col*self.spacing), 0, (col*self.spacing), self.height
            self.C.create_line(coord, fill="gray", tag="grid")
            i += 1
        #Create start point
        xNorm = int(self.navigationPoints[0][1])
        yNorm = int(self.navigationPoints[0][0])
        coord = (xNorm*self.spacing), (yNorm*self.spacing), ((xNorm+1)*self.spacing), ((yNorm+1)*self.spacing)
        self.C.create_oval(coord, fill="blue")
        #Create end point
        xNorm = int(self.navigationPoints[1][1])
        yNorm = int(self.navigationPoints[1][0])
        coord = (xNorm*self.spacing), (yNorm*self.spacing), ((xNorm+1)*self.spacing), ((yNorm+1)*self.spacing)
        self.C.create_oval(coord, fill="green")
        return i

    def canvas_leftClick(self,event):
        x, y = event.x, event.y
        xNorm = int(x/self.spacing)
        yNorm = int(y/self.spacing)
        if (self.var.get() == 1):   #if trying to place obstacle
            #if something is already there
            if (self.obstacleMap.count((yNorm,xNorm)) or self.navigationPoints[0]==[yNorm,xNorm] or self.navigationPoints[1]==[yNorm,xNorm]):
                return
            self.obstacleMap.append((yNorm,xNorm))
            coord = (xNorm*self.spacing), (yNorm*self.spacing), ((xNorm+1)*self.spacing), ((yNorm+1)*self.spacing)
            self.C.create_oval(coord, fill="black")
        elif (self.var.get() == 2): #if trying to place start point
            #if something is already there
            if (self.obstacleMap.count((yNorm,xNorm)) or self.navigationPoints[0]==[yNorm,xNorm] or self.navigationPoints[1]==[yNorm,xNorm]):
                return
            #delete old start point and add new
            row = self.navigationPoints[0][0]*self.spacing+(self.spacing/2)
            col = self.navigationPoints[0][1]*self.spacing+(self.spacing/2)
            self.C.delete(self.C.find_closest(col, row, start="grid"))
            self.navigationPoints[0] = [yNorm,xNorm]
            coord = (xNorm*self.spacing), (yNorm*self.spacing), ((xNorm+1)*self.spacing), ((yNorm+1)*self.spacing)
            self.C.create_oval(coord, fill="blue")

    def canvas_rightClick(self,event):
        x, y = event.x, event.y
        xNorm = int(x/self.spacing)
        yNorm = int(y/self.spacing)
        if (self.var.get() == 1):
            if (self.obstacleMap.count((yNorm,xNorm))):
                target = self.C.find_closest(x, y, start="grid")
                if (target[0] > self.index):    #make sure the target is an obstacle
                    self.C.delete(target)
                self.obstacleMap.remove((yNorm,xNorm))
            else:
                return
        elif (self.var.get() == 2):
            if (self.obstacleMap.count((yNorm,xNorm)) or self.navigationPoints[0]==[yNorm,xNorm] or self.navigationPoints[1]==[yNorm,xNorm]):
                return
            #delete old end point and add new
            row = self.navigationPoints[1][0]*self.spacing+(self.spacing/2)
            col = self.navigationPoints[1][1]*self.spacing+(self.spacing/2)
            self.C.delete(self.C.find_closest(col, row, start="grid"))
            self.navigationPoints[1] = [yNorm,xNorm]
            coord = (xNorm*self.spacing), (yNorm*self.spacing), ((xNorm+1)*self.spacing), ((yNorm+1)*self.spacing)
            self.C.create_oval(coord, fill="green")
            
    def option_sel(self):
       selection = self.var.get()
       print(selection)
       #label.config(text = selection)

    def startSimulation(self,event):
        w = int(self.width/self.spacing)
        h = int(self.height/self.spacing)
        graph = alg.AStarGraph(obstacles=self.obstacleMap,width=w,height=h)
        result, cost = alg.AStarSearch(self.navigationPoints,(0,0), graph)
        print ("route", result)
        print ("cost", cost)
        for i in range(len(result)-1):
            coord = (result[i][1]*self.spacing+self.spacing/2, result[i][0]*self.spacing+self.spacing/2,
                    result[i+1][1]*self.spacing+self.spacing/2, result[i+1][0]*self.spacing+self.spacing/2)
            self.C.create_line(coord, fill="red")




if __name__=="__main__":
    sim = simulation()
    sim.openSimulation()