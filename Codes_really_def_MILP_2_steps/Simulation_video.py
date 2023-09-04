import pygame

from pygame.locals import *

def run():
    FPS = 8
    pygame.init()
    fpsClock=pygame.time.Clock()

    number_of_parking_lanes=6
    number_of_spaces=5
    unit_height=150
    unit_width =100

    SCREEN_WIDTH, SCREEN_HEIGHT = (number_of_parking_lanes+2)*unit_width, (number_of_spaces+1)*unit_height
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
    surface = pygame.Surface(screen.get_size())
    surface = surface.convert()
    surface.fill((255,255,255))


    class Lane(object):
        def __init__(self, lane_number):
            self.font = pygame.font.Font(None, 50)
            self.length = 5
            self.color = (0,0,0)
            self.lane_number= lane_number
            self.positions =  [((lane_number+1)*unit_width, unit_height)]

        def draw(self, surf):
            pygame.draw.rect(surf, self.color, (self.positions[0][0], self.positions[0][1], unit_width, number_of_spaces*unit_height),3)
            pygame.draw.rect(surf, self.color, (self.positions[0][0], 0, unit_width, unit_height),3)
            surface.blit(self.font.render('N°' + str(self.lane_number), True, (255,0,0)), (self.positions[0][0]+unit_width/4, unit_height/2, unit_width, unit_height))

    class Parking_space(object):
        def __init__(self, lane_number, space_number):
            self.length = 5
            self.color = (0,0,0)
            self.lane_number= lane_number
            self.space_number= space_number

        def draw(self, surf):
            pygame.draw.rect(surf, self.color, ((self.lane_number+1)*unit_width, (self.space_number+1)*unit_height, unit_width, unit_height),1)


    if __name__ == '__main__':
        time_limit=100
        Parking_lanes=[]
        Parking_spaces=[]
        for n in range(number_of_parking_lanes):
            Parking_lanes.append(Lane(n))
            for s in range(number_of_spaces):
                Parking_spaces.append(Parking_space(n, s))

        time=0
        running=True
        while running and time <time_limit:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            surface.fill((255,255,255))
            #snake.move()
            for lane in Parking_lanes:
                lane.draw(surface)

            for space in Parking_spaces:
                space.draw(surface)

            screen.blit(surface, (0,0))
            font = pygame.font.Font(None, 36)
            text = font.render(str(time), 1, (10, 10, 10))
            textpos = text.get_rect()
            textpos.centerx = 20
            surface.blit(text, textpos)
            screen.blit(surface, (0,0))

            pygame.display.flip()
            pygame.display.update()
            time+=1
            fpsClock.tick(FPS)
        if time ==time_limit:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
        pygame.display.quit()
        pygame.quit()