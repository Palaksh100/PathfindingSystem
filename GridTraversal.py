import pygame
import heapq;

CELL_SIZE=30
WHITE=(255,255,255)
BLACK=(0,0,0)
BLUE=(0,0,255)
GREEN=(0,255,0)
COLOR_MAP={0:WHITE,1:BLUE}

class Grid:

    def __init__(self,grid,start,goal):
        self.grid=grid
        self.start=start
        self.goal=goal
        self.width=len(grid[0])
        self.height=len(grid)
    def is_valid(self,x,y=None):
        if 0<=x<self.width and 0<=y<self.height:
            if self.grid[x][y]==0:
                return True
        return False
    def find_neighbours(self,position):
        x,y=position
        directions=[(-1,0),(0,1),(1,0),(0,-1)]
        for dx,dy in directions:
            X,Y=x+dx,y+dy
            if self.is_valid(X,Y):
                yield (X,Y)
    def heuristic(self,point):
        return abs(point[0]-self.start[0])+abs(point[1]-self.start[1])  
    
class Cell:
        def __init__(self,grid,x,y):
            self.x=x
            self.y=y
            self.grid=grid
            self.parent=None
            self.f=float('inf')
            self.g=float('inf')
            self.h=grid.heuristic((x,y))  

def constructpath(pos):
    path=[]
    while(pos):
        path.append((pos.x,pos.y))
        pos=pos.parent
    return path[::-1]

def traverse(grid):
    width,height=grid.width,grid.height 
    if not (grid.is_valid(grid.start[0],grid.start[1])) and (grid.is_valid(grid.goal[0],grid.goal[1])):
        return 
    Done= [[False for _ in range(width)] for _ in range(height)]
    Cells= [[Cell(grid,x,y) for x in range(width)] for y in range(height)]
    Cells[grid.goal[0]][grid.goal[1]].g=0

    open_list=[]
    heapq.heappush(open_list,(grid.heuristic(grid.goal),grid.goal))

    while open_list:
        _,current=heapq.heappop(open_list)
        Done[current[0]][current[1]]=True

        if current==grid.start:
            return constructpath(Cells[current[0]][current[1]])
        
        for x,y in grid.find_neighbours(current):
            if not Done[x][y]:
                new_g=Cells[current[0]][current[1]].g+1
                new_f=new_g+grid.heuristic((x,y))
                if(new_f<Cells[x][y].f):
                    Cells[x][y].g=new_g
                    Cells[x][y].f=new_f
                    Cells[x][y].parent=Cells[current[0]][current[1]]
                    heapq.heappush(open_list,(new_f,(x,y)))                       

def Visualize(grid):
    pygame.init()
    width,height=grid.width,grid.height
    Screen=pygame.display.set_mode((width*CELL_SIZE,height*CELL_SIZE))
    background = pygame.Surface((Screen.get_size()))
    background.fill(WHITE)

    for y in range(height):
        for x in range(width):
            color=COLOR_MAP[grid.grid[y][x]]
            pygame.draw.rect(background,color,pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    for i in range(width):
        pygame.draw.line(background, BLACK, (i * CELL_SIZE, 0), (i * CELL_SIZE, height * CELL_SIZE), 1)

    for i in range(height):
        pygame.draw.line(background, BLACK, (0, i * CELL_SIZE), (width * CELL_SIZE, i * CELL_SIZE), 1)

    Screen.unlock()
    Screen.blit(background,(0,0))
    pygame.display.flip()