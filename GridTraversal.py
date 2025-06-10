import pygame
import sys
import ast
import copy
import heapq

CELL_SIZE=30
TIMESCALE=0.1
WHITE=(255,255,255)
BLACK=(0,0,0)
BLUE=(0,0,255)
GREEN=(0,255,0)
RED=(225,0,0)
ORANGE=(255,165,0)
YELLOW=(225,225,0)
COLOR_MAP={0:WHITE,1:BLUE,2:RED,3:ORANGE}
FPS=4

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
        self.original_map=copy.deepcopy(self.map)        

    def is_valid(self,x,y,z=-1):
        if 0<=x<self.width and 0<=y<self.height:
            if self.grid[y][x]==0:
                if z>=0:
                    for dz in range(0,round(1/TIMESCALE)):
                        if z-dz>=0:
                            if self.map[z-dz][y][x]!=0:
                                return False
                        if z+dz<=round(self.max_time/TIMESCALE):
                            if self.map[z+dz][y][x]!=0:
                                return False
                if z<=round(self.max_time/TIMESCALE):
                    return True
        return False
    
    def find_neighbours(self,position_time):
        x,y,z=position_time
        directions=[(-1,0),(0,1),(1,0),(0,-1)]
        if self.is_valid(x,y,z+1):
                yield (x,y,z+1)
        for dx,dy in directions:
            X,Y=x+dx,y+dy
            if self.is_valid(X,Y,z+round(1/TIMESCALE)):
                yield (X,Y,z+round(1/TIMESCALE))
    def heuristic(self,point):
        return (abs(point[0]-self.goal[0])+abs(point[1]-self.goal[1]))*round(1/TIMESCALE)  
    
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
    def __init__(self,path):   # path=[[(t,p)],[t],[(t,d,t_interval)]]
        self.appear=sorted(path[0])     
        self.disappear=sorted(path[1])
        self.move=sorted(path[2])
        self.active=False
        self.position=None

    def restricted_positions(self,max_time):
        '''Returns positions of all active obstacles at each time along with blocked cells'''
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
    
class UnKnownDynamicObstacle:
    def __init__(self,position,velocity,z,grid_width,grid_height):   
        self.position=position
        self.velocity=velocity
        self.grid_width=grid_width
        self.grid_height=grid_height
        self.UpdateBlockageMap(z)

    def valid_pos(self,pos):
        return -1<pos[0]<self.grid_width and -1<pos[1]<self.grid_height

    def UpdateBlockageMap(self,z):
        self.blockage_map=dict()
        t=0
        pos=self.position
        while self.valid_pos(pos):
            self.blockage_map[z+t]=integral_set(pos)
            pos=AddCoordinate(self.position,ScalerMultiplication(self.velocity,t*TIMESCALE))
            t+=1
            self.blockage_map[z+t]=integral_set(pos)
            removable_pos=[]
            for P in self.blockage_map[z+t]:
                if not self.valid_pos(pos):
                    removable_pos.append(P)
            for P in removable_pos:
                self.blockage_map[z+t].remove(P)

    def UpdatePosition(self,time):
        self.position=exact_coordinate(AddCoordinate(self.position,ScalerMultiplication(self.velocity,time)))    
    
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
        path.append((pos.x,pos.y,pos.z))
        pos=pos.parent
    return path[::-1]

def traverse(grid,UserObstacles=[],start_pos=None,start_time=0):
    if start_pos:
        grid.start=start_pos
    width,height,max_time=grid.width,grid.height,grid.max_time

    grid.map=copy.deepcopy(grid.original_map)
    for obstacle in UserObstacles:
        blocked_points=obstacle.blockage_map
        for t in blocked_points:
            for x,y in blocked_points[t]:
                grid.map[t][y][x]=3

    if not ((grid.is_valid(grid.start[0],grid.start[1],start_time)) and (grid.is_valid(grid.goal[0],grid.goal[1]))):
        return 
    
    Done= [[[False for _ in range(width)] for _ in range(height)] for _ in range(round(max_time/TIMESCALE))]
    Cells= [[[Cell(grid,x,y,z) for x in range(width)] for y in range(height)] for z in range(round(max_time/TIMESCALE))]
    Cells[start_time][grid.start[1]][grid.start[0]].g=0

    open_list=[]
    heapq.heappush(open_list,(grid.heuristic(grid.start),grid.start+(start_time,)))
    L=[]

    while open_list:
        _,current=heapq.heappop(open_list)
        Done[current[2]][current[1]][current[0]]=True

        if (current[0],current[1])==grid.goal:
            L=constructpath(Cells[current[2]][current[1]][current[0]])
            break
        
        for x,y,z in grid.find_neighbours(current):
            if not Done[z][y][x]:
                new_g=Cells[current[2]][current[1]][current[0]].g+(z-current[2])
                new_f=new_g+grid.heuristic((x,y))
                if(new_f<Cells[z][y][x].f):
                    Cells[z][y][x].g=new_g
                    Cells[z][y][x].f=new_f
                    Cells[z][y][x].parent=Cells[current[2]][current[1]][current[0]]
                    heapq.heappush(open_list,(new_f,(x,y,z)))     
    return L
                  

def Visualize(grid,pos,UnknownDynamicObstacles,t):
    font=pygame.font.SysFont("Arial", 20)
    width,height=grid.width,grid.height
    Screen=pygame.display.set_mode((width*CELL_SIZE,height*CELL_SIZE))
    background = pygame.Surface((Screen.get_size()))
    background.fill(WHITE)
    pygame.draw.rect(background,COLOR_MAP[2],(-2*CELL_SIZE,-2*CELL_SIZE,CELL_SIZE,CELL_SIZE))
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
    for x in grid.obstacle_positions:
        if t in x:
            pygame.draw.rect(background,COLOR_MAP[2],(x[t][0]*CELL_SIZE,x[t][1]*CELL_SIZE,CELL_SIZE,CELL_SIZE))

    for obstacle in UnknownDynamicObstacles:
        pygame.draw.rect(background,COLOR_MAP[3],(obstacle.position[0]*CELL_SIZE,obstacle.position[1]*CELL_SIZE,CELL_SIZE,CELL_SIZE))

    text=font.render(str(round(t*TIMESCALE,1)),True,(50,0,50))

    Screen.unlock()
    Screen.blit(background,(0,0))
    Screen.blit(text,(0,0))
    pygame.display.flip()

def show_solution(grid):
    Clock=pygame.time.Clock()
    L=traverse(grid)
    pygame.init() 
    pos=(L[0][0],L[0][1])
    i=1
    UserObstacles=[]  # UserObstacls=[(p,v)]
    pause_next=False
    pause=False
    for t in range(round(grid.max_time/TIMESCALE)):
        if i==len(L):
            break
        while i < len(L) and t >= L[i][2]:
            pos = (L[i][0], L[i][1])
            i += 1
            if pause_next:
                pause_next=False
                pause=True

        Visualize(grid,pos,UserObstacles,t)

        if pause:
            for i in range(1,1+len(UserObstacles)):
                print(f"Obstacle-{i} -> Position:{UserObstacles[i-1].position},Velocity:{UserObstacles[i-1].velocity}")
            user_input=ast.literal_eval(input("Enter velocity change list(Obstacle number,new velocity) >>"))
            for data in user_input:
                UserObstacles[data[0]-1].velocity=data[1]
                UserObstacles[data[0]-1].UpdateBlockageMap(t)

            user_input=ast.literal_eval(input("Enter deletion list(Obstacle number) >>"))
            for index in user_input:
                UserObstacles.pop(index-1)

            user_input=ast.literal_eval(input("Enter addition list(Position,velocity) >>"))
            for obstacle in user_input:
                UserObstacles.append(UnKnownDynamicObstacle(obstacle[0],obstacle[1],t,grid.width,grid.height))
            L=traverse(grid,UserObstacles,pos,t)
            print(L)
            i=1
            pause=False

        for obstacle in UserObstacles:
            obstacle.UpdatePosition(TIMESCALE)
            if not(obstacle.valid_pos(obstacle.position)):
                UserObstacles.remove(obstacle)        

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key==pygame.K_p:
                    pause_next=True
                    
        Clock.tick(FPS)
    pygame.quit()
    sys.exit()

grid_data = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0]
]
obstacle_path1 = [
    [  # appear: [(time, position)]
        (0.0, (4.0, 2.0)),
        
    ],
    [  
        
    ],
    [  
        (10, (0.0, 1.0), 1.5)   
    ]
]
obstacle_path2 = [
    [  # appear: [(time, position)]
        (3.0, (0.0, 3.0)),
    ],
    [  
        3.5
    ],
    [  
        
    ]
]
obstacle1 = KnownDynamicObstacle(obstacle_path1)
obstacle2 = KnownDynamicObstacle(obstacle_path2)
grid = Grid(grid_data,[obstacle1,obstacle2], (0, 0), (4, 4),40)
show_solution(grid)
