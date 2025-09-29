from jelly import *
from shapes import *
import pygame as pg
import random, time

# initialize pygame
pg.init()

# define basic things like resolution, screen, clock, colours..
width, height = 1280, 720
screen = pg.display.set_mode((width, height))
clock = pg.time.Clock()
colours = ["red", "green", "blue", "purple", "orange"]

# index of player pulling point
q = 9
# player pulling strength
p = 30

# class that defines the player
# player follows the mouse
class Player(Body):
    # initialize the player
    def __init__(self, p, m, k, pts, sprs, clr='yellow', tag='player'):
        super().__init__(p, m, k, pts, sprs, clr, tag)
    # update the player by 1 frame
    def update(self, dt):
        # get the mouse position
        mouse = np.array(pg.mouse.get_pos(), float)
        # add a force pulling towards the mouse
        self.pts[q].f = (mouse - self.pts[q].p) * self.m * p
        # update the players body base
        super().update(dt)

# number of dummy bodies
n = 1
# universal spring constant
uk = 50
# strength of gravity
g = [0, 100]
# magnification for size
mag = 100
# list of dummy bodies
bodies = []

# create the player
player = Player([640, 360], 1, uk,
                battlecatpts * mag,
                battlecatsprs)
# add the player to bodies list
bodies.append(player)

# generate random dummies
for i in range(n):
    # create the new body
    body = Body([random.randint(50, 500), random.randint(50, 500)], 1, uk,
                battlecatpts * mag, battlecatsprs, random.choice(colours))
    # add the body to bodies list
    bodies.append(body)

# time before frame end
lt = time.time()
# exit trigger
xt = False

# update all bodies
def update_bodies(dt):
    for body in bodies:
        # add gravity to total force
        for pt in body.pts: pt.f += g
        # update
        body.update(dt)

# handle the collisions
def handle_collisions():
    # loop through all combinations to check
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            # detect and resolve the collision
            collide(bodies[i], bodies[j])

# draw all the bodies
def draw_bodies():
    for body in bodies:
        # get the points of the body
        pts = [pt.p for pt in body.pts]
        # draw the rope for the player
        if body.tag == "player":
            mouse = np.array(pg.mouse.get_pos(), float)
            pg.draw.line(screen, "brown", pts[q], mouse, 10)
            pg.draw.circle(screen, "red", mouse, 5)
        # draw the body
        pg.draw.polygon(screen, body.clr, pts)
        pg.draw.polygon(screen, "black", pts, 5)

# make bodies bounce off of the edges of the screen
def wall_collision():
    for body in bodies:
        for pt in body.pts:
            if pt.p[0] <= 0:
                pt.v[0] *= -0.5
                pt.p[0] = 0
            if pt.p[0] >= width:
                pt.v[0] *= -0.5
                pt.p[0] = width
            if pt.p[1] <= 0:
                pt.v[1] *= -0.5
                pt.p[1] = 0
            if pt.p[1] >= height:
                pt.v[1] *= -0.5
                pt.p[1] = height

# collision frame counter
c = 0

# main game loop
while True:
    # add to collision frame counter
    c += 1
    # calculate delta time
    dt = time.time() - lt
    # update time before frame time
    lt = time.time()
    # check for events
    for event in pg.event.get():
        if event.type == pg.QUIT: xt = True
    # exit if triggered
    if xt: break
    # update bodies
    update_bodies(dt)
    if c == 10:
        c = 0
        # handle collision on walls
        wall_collision()
        # handle the collision of bodies
        handle_collisions()
        # refresh the screen
    screen.fill((156, 0, 156))
    # draw updated bodies
    draw_bodies()
    # update the screen
    pg.display.flip()
    #clock.tick(60)

# exit at the end
pg.quit()