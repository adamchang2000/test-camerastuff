import matplotlib.pyplot as plt
import numpy as np
import argparse


def draw_camera_fov(origin, vector, magnitude, degrees):
    """
    Draw a camera FOV defined by:
    1) origin
    2) vector (center view direction, magnitude does not matter)
    3) magnitude
    4) degrees FOV
    """

    assert(np.linalg.norm(vector) > 0)

    vector = vector * magnitude / np.linalg.norm(vector)
    v_angle = np.arctan2( vector[1] ,vector[0] ) # starting angle

    # plot the arrow
    fig = plt.figure(1)
    ax = plt.gca()
    ax.arrow(origin[0], origin[1] ,vector[0], vector[1], 
        head_width=0.05, 
        head_length=0.1, 
        lw=2, 
        fc='#777777', 
        ec='#777777', 
        length_includes_head=True)

    # plot the sector line
    uncert = (degrees/2.) *np.pi / 180.
    r = np.linalg.norm( vector ) # length of vector
    t = np.linspace( v_angle - uncert , v_angle+uncert , 100) # angular range of sector
    x = r* np.cos(t) + origin[0] # sector x coords
    y = r* np.sin(t) + origin[1] # sector y coords

    endpoint1_x = [origin[0], x[0]]
    endpoint1_y = [origin[1], y[0]]

    endpoint2_x = [origin[0], x[-1]]
    endpoint2_y = [origin[1], y[-1]]

    plt.plot(endpoint1_x, endpoint1_y)
    plt.plot(endpoint2_x, endpoint2_y)

    ax.plot( x,y, lw=2, ls='--') # plot the sector
    ax.plot( origin[0], origin[1], 'o', ms=10, c='Limegreen' ) # plot the origin

    # adjust the figure
    ax.set_xlim(-190, 100)
    ax.set_ylim(-100, 100)
    ax.set_aspect('equal')
    #plt.show()

def main():
    # parser = argparse.ArgumentParser(description='Plot camera FOVs in 2D Space')
	# parser.add_argument('--filename', default='medical/medical_aruco_sampled.ply', help='file path to model')
	# parser.add_argument('--output_dir', default='data', help='output for dataset')

	# args = parser.parse_args()

    #test camera
    origin = np.array([[15.0, -10.0], [0., 0.]])
    vector = np.array([[1.0, 1.0], [-1., 0.]])
    magnitude = np.array([40, 20])
    degrees = np.array([60, 30])

    for o, v, m, d in zip(origin, vector, magnitude, degrees):
        draw_camera_fov(o, v, m, d)


    plt.show()

if __name__ == '__main__':
    main()