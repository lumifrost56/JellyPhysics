import numpy as np

# check if a point is in a body
def pib(pt, body):
    # get the position of the point
    p = pt.p
    # split the coordinate into x and y
    x, y = p
    # define bool inside
    inside = False
    # define contact edge
    cedge = None
    # define minimum overlap
    minoverlap = float('inf')
    for i in range(body.n):
        # get the 2 points of the edge
        pt1 = body.pts[i]
        pt2 = body.pts[(i + 1) % body.n]
        # get the positions of the 2 points
        p1 = pt1.p
        p2 = pt2.p
        # split the positions into x and y
        x1, y1 = p1
        x2, y2 = p2
        # check if the points x axis intersects the edge
        if min(y1, y2) < y <= max(y1, y2):
            xi = (y - y1) * (x2 - x1) / (y2 - y1) + x1
            if x < xi: inside = not inside
        # get delta position
        dp = p2 - p1
        # calculate the contact point on the edge
        lsq = np.dot(dp, dp)
        t = np.dot(p - p1, dp) / lsq
        t = np.clip(t, 0, 1)
        cp = p1 + t * dp
        # calculate the overlap
        overlap = np.linalg.norm(p - cp)
        # keep track of the smallest overlap
        if overlap < minoverlap:
            minoverlap = overlap
            cedge = [pt1, pt2, cp]
    # return whether the point is inside and the contact edge
    return inside, cedge

# resolve the overlap and collision
# pt is the point inside the body
# cedge is the contact edge
# e is the coefficient of restitution
def resolve_collision(pt, cedge, e=0.5):
    # unpack the contact edge
    pt1, pt2, cp = cedge
    # get delta position
    dp = pt.p - cp
    # length from pt to cp
    l = np.linalg.norm(dp)
    # length of the edge
    el = np.linalg.norm(pt2.p - pt1.p)
    # length from pt1 to cp
    l1 = np.linalg.norm(pt1.p - cp)
    # length from pt2 to cp
    l2 = np.linalg.norm(pt2.p - cp)
    # get total mass
    tm = pt.m + pt1.m + pt2.m
    if l > 0:
        # adjust the positions based on mass and distance
        pt.p -= dp * (pt1.m + pt2.m) / tm
        pt1.p += dp * (pt.m / tm) * (l2 / el)
        pt2.p += dp * (pt.m / tm) * (l1 / el)
        # normalize delta position
        ndp = dp / l
        # get relative velocity
        rv = pt.v - (pt1.v * l2 + pt2.v * l1) / el
        # get normalized relative velocity along ndp
        nrv = np.dot(rv, ndp) * ndp
        # calculate the impulse
        j = -(1 + e) * nrv / (1 / pt.m + (l2**2 + l1**2) / (pt1.m + pt2.m) / el**2)
        # apply the impulse to all points
        pt.v += j / pt.m
        pt1.v -= j * l2 / (pt1.m + pt2.m) / el
        pt2.v -= j * l1 / (pt1.m + pt2.m) / el

# collide 2 bodies
def collide(body1, body2):
    # if bounding boxes of the 2 bodies overlap they could be colliding
    if (body1.le <= body2.ri and
        body1.ri >= body2.le and
        body1.to <= body2.bo and
        body1.bo >= body2.to):
        # check if the 2 collided
        for pt in body1.pts:
            inside, cedge = pib(pt, body2)
            # resolve the collision
            if inside: resolve_collision(pt, cedge)
        for pt in body2.pts:
            inside, cedge = pib(pt, body1)
            # resolve the collision
            if inside: resolve_collision(pt, cedge)

# class that defines a mass point
# p is position and m is mass
class Point:
    # initialize the point
    def __init__(self, p, m):
        # set position and mass
        self.p = p
        self.m = m
        # set velocity and force
        self.v = np.zeros(2)
        self.f = np.zeros(2)
    # update the point by 1 frame using verlet integration
    # dt is delta time and dmp is the damping factor
    def update(self, dt, dmp=0.95):
        # get acceleration
        acc = self.f / self.m
        # update the position
        self.p += self.v * dt + 0.5 * acc * dt**2
        # get new acceleration
        nacc = self.f / self.m
        # update the velocity
        self.v += 0.5 * (acc + nacc) * dt
        # damp the velocity
        self.v *= dmp**dt
        # reset active forces
        self.f = np.zeros(2)

# class that defines a spring
# a and b are the 2 points the spring connects
# k is the spring constant
class Spring:
    # initialize the spring
    def __init__(self, a, b, k):
        # set a and b points
        self.a = a
        self.b = b
        # set the spring constant
        self.k = k
        # get the resting length
        self.rl = np.linalg.norm(self.b.p - self.a.p)
    # apply force to points a and b
    # dmp is the damping factor
    def apply_force(self, dmp=0.5):
        # get delta position
        dp = self.b.p - self.a.p
        # get the length between the points
        l = np.linalg.norm(dp)
        if l > 0:
            # get normalized delta position
            ndp = dp / l
            # calculate the force that should be applied
            f = self.k * (l - self.rl)
            f += np.dot(self.b.v - self.a.v, ndp) * dmp
            f *= ndp
            # apply the force
            self.a.f += f
            self.b.f -= f

# class that defines the frame of the body
class Frame:
    # initialize the frame
    def __init__(self, body):
        # set the body
        self.body = body
        # get the skeleton
        self.skl = [pt.p - self.body.p for pt in self.body.pts]
        # get the points of the frame
        self.pts = [Point(pt.p + 0, pt.m) for pt in self.body.pts]
        # create springs for the frame
        sprs = [[self.pts[i], self.body.pts[i]] for i in range(self.body.n)]
        self.sprs = [Spring(a, b, self.body.k * 2) for a, b in sprs]
        # update the frame
        self.update()
    # correct bodys shape by applying forces between the body and frame
    def apply_forces(self):
        for spr in self.sprs: spr.apply_force()
    # rotate the frame by angle theta
    def rotate(self, th):
        # get sine and cosine of theta
        sth = np.sin(th)
        cth = np.cos(th)
        for pt in self.pts:
            # get points relative position
            pt.p -= self.body.p
            # separate the points position into x and y
            x = pt.p[0]
            y = pt.p[1]
            # rotate x and y by th
            nx = x * cth - y * sth
            ny = x * sth + y * cth
            # update the points position
            pt.p = np.array([nx, ny], float) + self.body.p
    # update the frame (basically preparation for rotating)
    def update(self):
        # set all points on the same position
        for pt in self.pts: pt.p = self.body.p + 0
        # define the list for all thetas
        ths = []
        for i in range(self.body.n):
            # add the skeleton to shape the points
            self.pts[i].p += self.skl[i]
            # get relative positions of points of the frame (a) and the body (b)
            a = self.pts[i].p - self.body.p
            b = self.body.pts[i].p - self.body.p
            # get the angle between the two
            th = np.arctan2(b[1], b[0]) - np.arctan2(a[1], a[0])
            # add it to the thetas list
            ths.append(th)
        # get the average sine and cosine of all the thetas
        smean = np.mean([np.sin(th) for th in ths], axis=0)
        cmean = np.mean([np.cos(th) for th in ths], axis=0)
        # finally determine by which angle the frame should be rotated
        ath = np.arctan2(smean, cmean)
        # rotate the frame
        self.rotate(ath)

# class that defines the body
# p is position of the center of mass of body
# m is mass of the body
# k is spring constant of all the springs constructing the body
# pts is coordinates of the points of the body
# sprs is sets of 2 points of all the springs of the body
# clr is just the colour of the body
class Body:
    # initialize the body
    def __init__(self, p, m, k, pts, sprs, clr='', tag=''):
        # get the number of points
        self.n = len(pts)
        # set the position
        self.p = np.array(p, float)
        # set the mass and spring constant
        self.m, self.k = m, k
        # set the temporary points
        pts = np.array(pts, float)
        # position them relative to their center of mass
        pts -= np.mean(pts, axis=0)
        # add bodys position to the points
        pts += self.p
        # set the points
        self.pts = [Point(pt, self.m / self.n) for pt in pts]
        # set the temporary springs
        sprs = [[self.pts[a], self.pts[b]] for a, b in sprs]
        # set the springs
        self.sprs = [Spring(a, b, self.k) for a, b in sprs]
        # set the frame
        self.frame = Frame(self)
        # set the colour
        self.clr = clr
        # set the special tag
        self.tag = tag
        # update the bounding box
        self.update_box()
    # update the bounding box
    def update_box(self):
        # separate the x and y coordinates
        x = [pt.p[0] for pt in self.pts]
        y = [pt.p[1] for pt in self.pts]
        # update the coordinate for all edges
        self.le, self.ri = np.min(x), np.max(x)
        self.to, self.bo = np.min(y), np.max(y)
    # update the body
    # dt is delta time
    def update(self, dt):
        # get the new center of mass of the body
        self.p = np.mean([pt.p for pt in self.pts], axis=0)
        # update the frame
        self.frame.update()
        # try to reshape the body
        self.frame.apply_forces()
        # apply forces to every spring
        for spr in self.sprs: spr.apply_force()
        # update every point
        for pt in self.pts: pt.update(dt)
        # update the bounding box
        self.update_box()