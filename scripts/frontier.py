import queue


class frontier(object):
    def __init__(self):
        self.q_map = queue.Queue()
        self.q_frontier = queue.Queue()
        # these lists are np arrays of map size
        self.map_open_list = None
        self.map_close_list = None
        self.frontier_open_list = None
        self.frontier_close_list = None
        self.OccupancyGrid = None
        self.newFrontier = None
    
    def is_frontier(self, pixel):
        #TODO: check if a pixel is a frontier using self.map (occupancy grid)
        pass

    def adjacent(self, pixel):
        #TODO: returns all neiboring pixels
        neighbors = []
        pass
        return neighbors

    def has_one_open_neighbor(self,pixel):
        for p in self.adjacent(pixel):
            if self.map_open_list[p.x,py]: return True
        return False

    def update_map(self,map):
        self.map = map

    def step(self):
        if self.q_map.empty(): return False
        p = self.q_map.get()
        if map_close_list[p.x, p.y]:
            return self.step()
        if is_frontier(p.x,p.y):
            self.q_frontier = queue.Queue()
            self.newFrontier = []
            self.q_frontier.put(p)
            self.frontier_open_list[p.x,p.y] = True
            while not self.q_frontier.empty():
                q = self.q_frontier.get()
                if self.map_close_list[q.x,q.y] and self.frontier_close_list[q.x,q.y]:
                    continue
                if is_frontier(q):
                    self.newFrontier.append(q)
                    for w in self.adjacent(q):
                        if not (self.frontier_open_list[w.x,w.y] and self.frontier_close_list[w.x,w.y] and self.map_close_list[w.x,w.y]):
                            self.q_frontier.put(w)
                            self.frontier_open_list[w.x,w.y] = True
                self.frontier_close_list[q.x,q.y] = True
            for pixel in self.newFrontier:
                self.map_close_list[pixel.x,pixel.y] = True
        for v in self.adjacent(p):
            if (not (self.map_open_list[v.x,v.y] and self.map_close_list[v.x,v.y]) and self.has_one_open_neighbor(v)):
                self.q_map.put(v)
                self.map_open_list[v.x,v.y] = True
        self.map_close_list[p.x,p.y] = True
        return p