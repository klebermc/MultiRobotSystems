import numpy as np
from icecream import ic
import matplotlib.pyplot as plt

# Displacement-based formation control (see matlab/sim_formation_vel.m for
# the same approach in MATLAB) driving 9 nodes toward a target "pyramid"
# shape, with one node pinned to a waypoint path that the rest follow.

# Target relative (x, y, z) offsets for each of the 9 nodes in the pyramid
# formation, before scaling/centering below.
End_POS_pyramid = np.array([[0.0125, 0.2325,   0.2325,   0.0125,   0.1350, 0.2700, 0.1350,  0.0,    0.1350],
                            [0.2200, 0.2200,   0.0,      0.0,      0.2400, 0.1225, 0.0,     0.1225, 0.1225],
                            [0.0,    0.0,      0.0,      0.0,      0.1350, 0.1350, 0.1350,  0.1350, 0.2700]])
End_POS_pyramid[2,:]=End_POS_pyramid[2,:]+0.05
End_POS_pyramid=End_POS_pyramid*4
End_POS_pyramid=End_POS_pyramid.transpose()

dt=0.1
kp=0.1

assembling=False

#Intitilize figure
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# plt.grid()
# plt.xlim([0,2])
# plt.ylim([0,2])

## WP navigation of the ensemble
WP = np.array([[0,0,0.5],[0,1,0.5],[1,1,0.5],[1,0,0.5],[0,0,0.5]])

nextwp=1

# Fixing random state for reproducibility
np.random.seed(19680801)

# 3D - 9 nodes
N=9                # Nodes in the network, cardinallity of vertex set
n=3                # Variables of the state of each node
A=np.zeros([N,N])       # Ajacency Matrix

# Defines which node pairs exchange formation-error corrections with each
# other each step (the communication/sensing graph for consensus) — set to
# match the pyramid's structural edges, not a complete graph.
A[0,2]=1
A[0,4]=1                      
A[0,7]=1  
A[0,8]=1
A[1,4]=1 
A[1,5]=1
A[2,5]=1 
A[2,6]=1                                                  
A[3,6]=1 
A[3,7]=1 
A[4,5]=1 
A[4,6]=1 
A[4,7]=1 
A[4,8]=1 
A[5,6]=1 
A[5,8]=1 
A[6,7]=1 
A[6,8]=1
A[7,8]=1

A = A + np.transpose(A)
A_deg=A;       # Ajacency Matrix version with degree

maxT=100

x=np.zeros([int(maxT/dt)+1, N,n])
ic(x.shape)

for i in range(N):
    x[0,i]=[i/10,np.random.random(),1.5]

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


for t in np.arange(0, maxT, dt):
    k=round(t/dt)
    for i in range(N):
        # Figure plot of the nodes        
        ax.scatter(x[k,:,0], x[k,:,1], x[k,:,2], marker='x', color='r')
        ax.text(x[k,i,0]+0.0001,x[k,i,1]+0.0001, x[k,i,2], str(i))
        ax.plot([x[k,i,0],x[k,i,0]],[x[k,i,1],x[k,i,1]],[x[k,i,2],0],'k',linestyle='dashed', linewidth=1) 

        posi = x[k,i]
        u = x[1,1]*0

        posi_s = End_POS_pyramid[i]

        for j in range(N):
            posj = x[k,j]
            if i!=j and A[i,j]!=0:
                posj_s = End_POS_pyramid[j]
                # Displacement-consensus term: correct this node's position
                # so its offset from neighbor j matches the offset their
                # pyramid targets have from each other.
                u = u + kp*(posi - posj - posi_s + posj_s)
                if i>j:
                    ax.plot([x[k,i,0],x[k,j,0]],
                            [x[k,i,1],x[k,j,1]],
                            [x[k,i,2],x[k,j,2]],
                            'k', linestyle='-', linewidth=0.1)

            # Collision-avoidance term (Olfati-Saber-style sigma-norm
            # potential function) is disabled here — it depends on an
            # `Auxiliar` helper that isn't defined in this file/project, so
            # this block is left commented out rather than left broken.
            # if i!=j:
            #     # trying to avoid colisions
            #     diff = posj-posi;
            #     sigma_diff = Auxiliar.sigma_norm(diff);
            #     sigma_d = Auxiliar.sigma_norm(1);
            #     phi_alpha = Auxiliar.rho_h(sigma_diff/sigma_int_range)*Auxiliar.sigma_1(sigma_diff-sigma_d);
            #     nij= diff/sqrt(1+epslon*norm(diff,2)^2);
            #     # do not avoid colisions if it is an assembly stage
            #     if assembling:
            #         u = u - phi_alpha*nij

        u=-1*u

        # Pinning controller: node 8 alone is driven toward the current
        # waypoint, dragging the rest of the formation along via the
        # consensus term above (the other 8 nodes just follow node 8).
        if i==8:
            u = u + 2e0*(WP[nextwp]-x[k,i])
            ic(np.linalg.norm(WP[nextwp]-x[k,i]))

        x[k+1,i] = posi + dt*u

    # Once the pinned node is close enough to its waypoint, start shrinking
    # the pyramid's target offsets toward zero (nodes converge together)
    # rather than moving to a new waypoint — an "assembly" stage.
    if  np.linalg.norm(WP[nextwp]-x[k,8])  < 0.2 and \
        np.linalg.norm(End_POS_pyramid[0]-End_POS_pyramid[1]) > 0.2:
        End_POS_pyramid=End_POS_pyramid*0.999
        assembling=True
        ic()

    plt.draw()
    plt.pause(0.01)
    plt.cla()