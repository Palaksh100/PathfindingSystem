import pygame

CELL_SIZE=30
WHITE=(255,255,255)
BLACK=(0,0,0)
BLUE=(0,0,255)
GREEN=(0,255,0)
COLOR_MAP={0:WHITE,1:BLUE,2:GREEN}



def Visualize(grid):
    pygame.init()
    width,height=len(grid[0]),len(grid)
    Screen=pygame.display.set_mode((width*CELL_SIZE,height*CELL_SIZE))
    background = pygame.Surface((Screen.get_size()))
    background.fill(WHITE)

    for y in range(height):
        for x in range(width):
            color=COLOR_MAP[grid[y][x]]
            pygame.draw.rect(background,color,pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    for i in range(width):
        pygame.draw.line(background, BLACK, (i * CELL_SIZE, 0), (i * CELL_SIZE, height * CELL_SIZE), 1)

    for i in range(height):
        pygame.draw.line(background, BLACK, (0, i * CELL_SIZE), (width * CELL_SIZE, i * CELL_SIZE), 1)

    Screen.unlock()
    Screen.blit(background,(0,0))
    pygame.display.flip()