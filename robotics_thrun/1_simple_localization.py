def sense(p, Z):
    q = []
    for i in range(len(p)):
        hit = (z == world[i])
        q.append(p[i]*(hit*pHit + (1-hit)*pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i]/s
    return q

def move(p,U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U)%len(p)]
        s = s+pOvershoot*p[(i-U-1)%len(p)]
        s = s +pUndershooot* p[(i-U+1)%len(p)]
        q.append(s)
    return q

def localize(colors, measurements, motions, sensor_right,p_move):
    pinit = 1.0/float(len(colors))/float(len(colors[0]))
    p =[[pinit for row in range(len(colors[0]))] for col in range(len(colors))]
    return p


def show(p):
    rows = ['['+','.join(map(lambda x: '{0:.5f}'.format(x),r))+']' for r in p]
    print('['+',\n'.join(rows)+']')


colors = [['G','G','G'],
['G','R','G'],
['G','G','G']
]

measurements =['R']

motions=[[0,0]]
sensor_right = 1.0
p_move = 1.0

p = localize(colors,measurements,motions,sensor_right=sensor_right,p_move=p_move)
show(p)