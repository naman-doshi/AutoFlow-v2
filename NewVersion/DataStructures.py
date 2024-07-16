class DSU:
    '''
    Disjoint Set Union data structure for checking if two intersections are connected in the graph, in amortised O(1) time.
    
    Used for connecting all components of the procedurally generated road network to each other to make every node
    traversable from every other node.
    '''
    def __init__(self, elements):
        self.comp = [-1] * elements

    def union(self, a, b):
        a = self.repr(a)
        b = self.repr(b)
        if a == b:
            return True
        if -self.comp[a] < -self.comp[b]:
            a, b = b, a
        self.comp[a] += self.comp[b]
        self.comp[b] = a
        return False

    def is_repr(self, x):
        return self.comp[x] < 0

    def size(self, x):
        return -self.comp[self.repr(x)]

    def num_comps(self):
        return sum(1 for x in self.comp if x < 0)

    def repr(self, x):
        if self.comp[x] < 0:
            return x
        par = self.comp[x]
        self.comp[x] = self.repr(par)
        return self.comp[x]

    def connected(self, a, b):
        return self.repr(a) == self.repr(b)

class Lazy:
    def __init__(self, v=0, inc=True):
        self.v = v
        self.inc = inc

    def __iadd__(self, other):
        if other.inc:
            self.v += other.v
        else:
            self.v = other.v
            self.inc = False
        return self

class Node:
    def __init__(self, mx=float('-inf'), sum=0):
        self.mx = mx
        self.sum = sum

    def __add__(self, other):
        return Node(max(self.mx, other.mx), self.sum + other.sum)

    def upd(self, lazy, l, r):
        if lazy.inc:
            self.mx += lazy.v
            self.sum += lazy.v * (r - l + 1)
        else:
            self.mx = lazy.v
            self.sum = lazy.v * (r - l + 1)

class LazySeg:
    def __init__(self, SZ, NID, UID):
        self.NID = NID
        self.UID = UID
        self.SZ = SZ
        self.seg = [NID] * (2 * SZ)
        self.lazy = [UID] * (2 * SZ)

    def pull(self, i):
        self.seg[i] = self.seg[2 * i] + self.seg[2 * i + 1]

    def push(self, i, l, r):
        self.seg[i].upd(self.lazy[i], l, r)
        if l != r:
            for j in range(2):
                self.lazy[2 * i + j] += self.lazy[i]
        self.lazy[i] = self.UID

    def build(self):
        for i in range(self.SZ - 1, 0, -1):
            self.pull(i)

    def upd(self, lo, hi, val, i=1, l=0, r=None):
        if r is None:
            r = self.SZ - 1
        self.push(i, l, r)
        if r < lo or l > hi:
            return
        if lo <= l and r <= hi:
            self.lazy[i] += val
            self.push(i, l, r)
            return
        m = (l + r) // 2
        self.upd(lo, hi, val, 2 * i, l, m)
        self.upd(lo, hi, val, 2 * i + 1, m + 1, r)
        self.pull(i)

    def query(self, lo=0, hi=None, i=1, l=0, r=None):
        if hi is None:
            hi = self.SZ - 1
        if r is None:
            r = self.SZ - 1
        self.push(i, l, r)
        if r < lo or l > hi:
            return self.NID
        if lo <= l and r <= hi:
            return self.seg[i]
        m = (l + r) // 2
        return self.query(lo, hi, 2 * i, l, m) + self.query(lo, hi, 2 * i + 1, m + 1, r)

    def __getitem__(self, i):
        return self.seg[i + self.SZ]

    def __setitem__(self, i, value):
        self.seg[i + self.SZ] = value


# INF = float('inf')
# SZ = 1 << 20
# n = 1000
# tree = LazySeg(SZ, Node(-INF, 0), Lazy(0, True))
# for i in range(n):
#     tree[i] = Node(0, 0)
# tree.build()

# tree.query(l, r).mx  # maximum of [l, r]
# tree.upd(l, r, Lazy(1, True))  # increments [l, r] by 1
# tree.upd(l, r, Lazy(0, False))  # sets [l, r] to 0
