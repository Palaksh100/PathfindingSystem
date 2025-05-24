import pygame
import heapq

CELL_SIZE=30
TIMESCALE=0.1
WHITE=(255,255,255)
BLACK=(0,0,0)
BLUE=(0,0,255)
GREEN=(0,255,0)
RED=(225,0,0)
YELLOW=(225,225,0)
COLOR_MAP={0:WHITE,1:BLUE,2:RED}
FPS=0.8
Clock=pygame.time.Clock()

class Grid:

    def __init__(self,grid,start,goal):
        self.grid=grid
        self.start=start
        self.goal=goal
        self.width=len(grid[0])
        self.height=len(grid)
    def is_valid(self,x,y=None):
        if 0<=x<self.width and 0<=y<self.height:
            if self.grid[y][x]==0:
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
class KnownDynamicObstacle:
    def __init__(self,grid,path):   # path=[[(t,p)],[t],[(t,p,t_interval)]]
        self.grid=grid
        self.appear=sorted(path[0])     
        self.disappear=sorted(path[1])
        self.move=sorted(path[2])
        self.active=False
        self.position=None

    def restricted_positions(self,max_time):
        def find_obstacle_positions_move(self,move_event,time_position_map):
            time=move_event[0]
            interval=move_event[2]
            initial_position=self.position
            for t in frange(time+TIMESCALE,time+interval+TIMESCALE,TIMESCALE):
                self.position=exact_coordinate(AddCoordinate(initial_position,ScalerMultiplication(move_event[1],(t-time)/interval)))
                time_position_map[t]=self.position
            return time_position_map 
        
        time_position_map=dict() # time_position_map={t:{p}}
        index_a=index_d=index_m=0
        k=frange(0,max_time,TIMESCALE)
        for t in k:
            print(t)
            if not(self.active) and index_a<len(self.appear):
                if Equal(t,self.appear[index_a][0]):
                    self.active=True
                    self.position=self.appear[index_a][1]
                    index_a+=1
            if self.active:
                time_position_map[t]=self.position
                if index_d<len(self.disappear):
                    if Equal(t,self.disappear[index_d]):
                        self.active=False
                        self.position=None
                        index_d+=1
                        continue
                if index_m<len(self.move):
                    if Equal(t,self.move[index_m][0]):
                        time_position_map=find_obstacle_positions_move(self,self.move[index_m],time_position_map)
                        
                        for i in k:
                            if Equal(i,t+self.move[index_m][2]-TIMESCALE):
                                break   
                        index_m+=1
        return time_position_map
    
def exact(t,i=8):
    return round(t,i)  

def exact_coordinate(p):
    return (exact(p[0]),exact(p[1]))

def frange(start, stop, step):
    while exact(start) < exact(stop):
        yield exact(start)  # rounding avoids floating point accumulation errors
        start += step

def Equal(a,b,i=1e-7):
    return abs(a-b) < i

def Abs(p):
    return ((p[0])**2+(p[1])**2)**0.5
def AddCoordinate(p1,p2):
    return (p1[0]+p2[0],p1[1]+p2[1])
def ScalerMultiplication(p,s):
    return (s*p[0],s*p[1])

def constructpath(pos):
    path=[]
    while(pos):
        path.append((pos.x,pos.y))
        pos=pos.parent
    return path

def traverse(grid):
    width,height=grid.width,grid.height 
    if not ((grid.is_valid(grid.start[0],grid.start[1])) and (grid.is_valid(grid.goal[0],grid.goal[1]))):
        return 
    Done= [[False for _ in range(width)] for _ in range(height)]
    Cells= [[Cell(grid,x,y) for x in range(width)] for y in range(height)]
    Cells[grid.goal[1]][grid.goal[0]].g=0

    open_list=[]
    heapq.heappush(open_list,(grid.heuristic(grid.goal),grid.goal))

    while open_list:
        _,current=heapq.heappop(open_list)
        Done[current[1]][current[0]]=True

        if current==grid.start:
            L=constructpath(Cells[current[1]][current[0]])
            print(L)
            break
        
        for x,y in grid.find_neighbours(current):
            if not Done[y][x]:
                new_g=Cells[current[1]][current[0]].g+1
                new_f=new_g+grid.heuristic((x,y))
                if(new_f<Cells[y][x].f):
                    Cells[y][x].g=new_g
                    Cells[y][x].f=new_f
                    Cells[y][x].parent=Cells[current[1]][current[0]]
                    heapq.heappush(open_list,(new_f,(x,y)))    
    print(L)  
    pygame.init()   
    for pos in L:
        Clock.tick(FPS)
        Visualize(grid,pos)
                  

def Visualize(grid,pos):
    for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
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
    pygame.draw.circle(background,YELLOW,(CELL_SIZE*(0.5+pos[0]),CELL_SIZE*(0.5+pos[1])),CELL_SIZE/2)
    pygame.draw.circle(background,GREEN,(CELL_SIZE*(0.5+grid.goal[0]),CELL_SIZE*(0.5+grid.goal[1])),CELL_SIZE/2)
    Screen.unlock()
    Screen.blit(background,(0,0))
    pygame.display.flip()

grid_data = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0]
]
obstacle_path = [
    [  # appear: [(time, position)]
        (0.0, (1.0, 1.0)),
        (5.0, (3.0, 1.0))
    ],
    [  # disappear: [time]
        2.0,
        7.0
    ],
    [  # move: [(start_time, direction_vector, duration)]
        (0.5, (1.0, 0.0), 1.0),  # move right for 1s
        (5.5, (0.0, 1.0), 1.5)   # move down for 1.5s
    ]
]
grid = Grid(grid_data, (0, 0), (4, 4))
obstacle = KnownDynamicObstacle(grid, obstacle_path)
blocked_positions = obstacle.restricted_positions(max_time=10.0)

for t in sorted(blocked_positions.keys()):
    print(f"Time {t:.1f}: Position {blocked_positions[t]}")

