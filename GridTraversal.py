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
    def __init__(self,grid,obstacles,start,goal,max_time):
        self.grid=grid
        self.start=start
        self.goal=goal
        self.width=len(grid[0])
        self.height=len(grid)
        self.max_time=max_time

        obstacle_data=[obstacle.restricted_positions(max_time) for obstacle in obstacles]
        self.obstacle_positions=[x[0] for x in obstacle_data]
        self.map = [ [row[:] for row in self.grid] for _ in range(round(max_time / TIMESCALE + 1)) ]
        for obstacle in obstacle_data:
            blocked_points=obstacle[1]
            for t in blocked_points:
                for x,y in blocked_points[t]:
                    self.map[t][y][x]=2

    def is_valid(self,x,y,z=-1):
        if 0<=x<self.width and 0<=y<self.height:
            if self.grid[y][x]==0:
                if z>=0:
                    for dz in range(0,round(1/TIMESCALE)):
                        if z-dz>=0:
                            if self.map[z-dz][y][x]==2:
                                return False
                        if z+dz<=round(grid.max_time/TIMESCALE):
                            if self.map[z+dz][y][x]==2:
                                return False
                if z<=round(grid.max_time/TIMESCALE):
                    return True
        return False
    def find_neighbours(self,position_time):
        x,y,z=position_time
        directions=[(-1,0),(0,1),(1,0),(0,-1)]
        for dx,dy in directions:
            X,Y=x+dx,y+dy
            if self.is_valid(X,Y,z+round(1/TIMESCALE)):
                yield (X,Y,z+round(1/TIMESCALE))
            if self.is_valid(x,x,z+1):
                yield (x,y,z+1)
    def heuristic(self,point):
        return abs(point[0]-self.goal[0])+abs(point[1]-self.goal[1])  
    
class Cell:
        def __init__(self,grid,x,y,z):
            self.x=x
            self.y=y
            self.z=z
            self.grid=grid
            self.parent=None
            self.f=float('inf')
            self.g=float('inf')
            self.h=grid.heuristic((x,y))  
class KnownDynamicObstacle:
    def __init__(self,path):   # path=[[(t,p)],[t],[(t,p,t_interval)]]
        self.appear=sorted(path[0])     
        self.disappear=sorted(path[1])
        self.move=sorted(path[2])
        self.active=False
        self.position=None

    def restricted_positions(self,max_time):
        def find_obstacle_positions_move(self,move_event,time_position_map):
            time=round(move_event[0]/TIMESCALE)
            interval=round(move_event[2]/TIMESCALE)
            initial_position=self.position
            for t in range(time+1,time+interval+1):
                self.position=exact_coordinate(AddCoordinate(initial_position,ScalerMultiplication(move_event[1],(t-time)/interval)))
                time_position_map[t]=self.position
            return time_position_map 
        
        time_position_map=dict() # time_position_map={t:{p}}
        index_a=index_d=index_m=0
        max_t=round(max_time/TIMESCALE)
        t=0
        while t<max_t:
            if not(self.active) and index_a<len(self.appear):
                if t==round(self.appear[index_a][0]/TIMESCALE):
                    self.active=True
                    self.position=self.appear[index_a][1]
                    index_a+=1
            if self.active:
                time_position_map[t]=self.position
                if index_d<len(self.disappear):
                    if t==round(self.disappear[index_d]/TIMESCALE):
                        self.active=False
                        self.position=None
                        index_d+=1
                        continue
                if index_m<len(self.move):
                    if t==round(self.move[index_m][0]/TIMESCALE):
                        time_position_map=find_obstacle_positions_move(self,self.move[index_m],time_position_map)
                        t+=round(self.move[index_m][2]/TIMESCALE)-1  
                        index_m+=1
            t+=1

        blockage_map=dict()
        for t in time_position_map:
            blockage_map[t]=integral_set(time_position_map[t])
        return (time_position_map,blockage_map)
    
def integral_set(position):
    x,y=position[0],position[1]
    ix,iy=int(x//1),int(y//1)
    s={(ix,iy)}
    if not(Equal(x%1,0)):
        s.add((ix+1,iy))
        if not(Equal(y%1,0)):
            s.add((ix+1,iy+1))
    if not(Equal(y%1,0)):
        s.add((ix,iy+1))
        if  not(Equal(x%1,0)):
            s.add((ix+1,iy+1))
    return s
    
def exact(t,i=6):
    return round(t,i)  

def exact_coordinate(p):
    return (exact(p[0]),exact(p[1]))

def Equal(a,b,i=1e-6):
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
    return path[::-1]

def traverse(grid):
    width,height,max_time=grid.width,grid.height,grid.max_time
    if not ((grid.is_valid(grid.start[0],grid.start[1],0)) and (grid.is_valid(grid.goal[0],grid.goal[1]))):
        return 
    Done= [[[False for _ in range(width)] for _ in range(height)] for _ in range(round(max_time/TIMESCALE))]
    Cells= [[[Cell(grid,x,y,z) for x in range(width)] for y in range(height)] for z in range(round(max_time/TIMESCALE))]
    Cells[0][grid.start[1]][grid.start[0]].g=0

    open_list=[]
    heapq.heappush(open_list,(grid.heuristic(grid.start),grid.start+(0,)))
    L=[]

    while open_list:
        _,current=heapq.heappop(open_list)
        Done[current[2]][current[1]][current[0]]=True

        if (current[0],current[1])==grid.goal:
            L=constructpath(Cells[current[2]][current[1]][current[0]])
            break
        
        for x,y,z in grid.find_neighbours(current):
            if not Done[z][y][x]:
                new_g=Cells[current[2]][current[1]][current[0]].g+1
                new_f=new_g+grid.heuristic((x,y))
                if(new_f<Cells[z][y][x].f):
                    Cells[z][y][x].g=new_g
                    Cells[z][y][x].f=new_f
                    Cells[z][y][x].parent=Cells[current[2]][current[1]][current[0]]
                    heapq.heappush(open_list,(new_f,(x,y,z)))     
    # pygame.init()   
    # for pos in L:
    #     Clock.tick(FPS)
    #     Visualize(grid,pos)
    return L
                  

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
obstacle = KnownDynamicObstacle(obstacle_path)
grid = Grid(grid_data,[obstacle], (0, 0), (4, 4),16)
print(traverse(grid))
