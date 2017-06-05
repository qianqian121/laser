#!/usr/bin/python

def lidar_to_2d_front_view(points,
                           v_res,
                           h_res,
                           v_fov,
                           val="depth",
                           cmap="jet",
                           saveto=None,
                           y_fudge=0.0
                           ):
    """ Takes points in 3D space from LIDAR data and projects them to a 2D
        "front view" image, and saves that image.

    Args:
        points: (np array)
            The numpy array containing the lidar points.
            The shape should be Nx4
            - Where N is the number of points, and
            - each point is specified by 4 values (x, y, z, reflectance)
        v_res: (float)
            vertical resolution of the lidar sensor used.
        h_res: (float)
            horizontal resolution of the lidar sensor used.
        v_fov: (tuple of two floats)
            (minimum_negative_angle, max_positive_angle)
        val: (str)
            What value to use to encode the points that get plotted.
            One of {"depth", "height", "reflectance"}
        cmap: (str)
            Color map to use to color code the `val` values.
            NOTE: Must be a value accepted by matplotlib's scatter function
            Examples: "jet", "gray"
        saveto: (str or None)
            If a string is provided, it saves the image as this filename.
            If None, then it just shows the image.
        y_fudge: (float)
            A hacky fudge factor to use if the theoretical calculations of
            vertical range do not match the actual data.

            For a Velodyne HDL 64E, set this value to 5.
    """

    # DUMMY PROOFING
    assert len(v_fov) ==2, "v_fov must be list/tuple of length 2"
    assert v_fov[0] <= 0, "first element in v_fov must be 0 or negative"
    assert val in {"depth", "height", "reflectance"}, \
        'val must be one of {"depth", "height", "reflectance"}'


    x_lidar = points[:, 0]
    y_lidar = points[:, 1]
    z_lidar = points[:, 2]
    r_lidar = points[:, 3] # Reflectance
    # Distance relative to origin when looked from top
    d_lidar = np.sqrt(x_lidar ** 2 + y_lidar ** 2)
    # Absolute distance relative to origin
    # d_lidar = np.sqrt(x_lidar ** 2 + y_lidar ** 2, z_lidar ** 2)

    v_fov_total = -v_fov[0] + v_fov[1]

    # Convert to Radians
    v_res_rad = v_res * (np.pi/180)
    h_res_rad = h_res * (np.pi/180)

    # PROJECT INTO IMAGE COORDINATES
    x_img = np.arctan2(-y_lidar, x_lidar)/ h_res_rad
    y_img = np.arctan2(z_lidar, d_lidar)/ v_res_rad

    # SHIFT COORDINATES TO MAKE 0,0 THE MINIMUM
    x_min = -360.0 / h_res / 2  # Theoretical min x value based on sensor specs
    x_img -= x_min              # Shift
    x_max = 360.0 / h_res       # Theoretical max x value after shifting

    y_min = v_fov[0] / v_res    # theoretical min y value based on sensor specs
    y_img -= y_min              # Shift
    y_max = v_fov_total / v_res # Theoretical max x value after shifting

    y_max += y_fudge            # Fudge factor if the calculations based on
                                # spec sheet do not match the range of
                                # angles collected by in the data.

    # WHAT DATA TO USE TO ENCODE THE VALUE FOR EACH PIXEL
    if val == "reflectance":
        pixel_values = r_lidar
    elif val == "height":
        pixel_values = z_lidar
    else:
        pixel_values = -d_lidar

    # PLOT THE IMAGE
    cmap = "jet"            # Color map to use
    dpi = 100               # Image resolution
    fig, ax = plt.subplots(figsize=(x_max/dpi, y_max/dpi), dpi=dpi)
    ax.scatter(x_img,y_img, s=1, c=pixel_values, linewidths=0, alpha=1, cmap=cmap)
    ax.set_axis_bgcolor((0, 0, 0)) # Set regions with no points to black
    ax.axis('scaled')              # {equal, scaled}
    ax.xaxis.set_visible(False)    # Do not draw axis tick marks
    ax.yaxis.set_visible(False)    # Do not draw axis tick marks
    plt.xlim([0, x_max])   # prevent drawing empty space outside of horizontal FOV
    plt.ylim([0, y_max])   # prevent drawing empty space outside of vertical FOV

    if saveto is not None:
        fig.savefig(saveto, dpi=dpi, bbox_inches='tight', pad_inches=0.0)
    else:
        fig.show()


import matplotlib.pyplot as plt
import numpy as np

def load_file_points(filename):
    from array import array
    input_file = open(filename, 'r')
    float_array = array('f')
    float_array.fromstring(input_file.read())
    # x = float_array[0::4]
    # y = float_array[1::4]
    # z = float_array[2::4]
    # r = float_array[3::4]
    points = np.asarray(float_array)
    return points


def _getLidarPoints(frameId, dataset = None):
    filename = os.path.join(dataset, '%010d.bin' % frameId)
    if not os.path.isfile(filename):
        # no data
        return None
    # read matrix
    data = np.fromfile(filename, np.float32)
    # reshape to points with 4 coordinates each
    data = data.reshape(data.shape[0] // 4, 4)

    return data

def lidar_show(lidar):
    HRES = 0.35         # horizontal resolution (assuming 20Hz setting)
    VRES = 0.4          # vertical res
    VFOV = (-24.9, 2.0) # Field of view (-ve, +ve) along vertical axis
    Y_FUDGE = 5         # y fudge factor for velodyne HDL 64E

    lidar_to_2d_front_view(lidar, v_res=VRES, h_res=HRES, v_fov=VFOV, val="depth",
                           saveto="/tmp/lidar_depth.png", y_fudge=Y_FUDGE)

    lidar_to_2d_front_view(lidar, v_res=VRES, h_res=HRES, v_fov=VFOV, val="height",
                           saveto="/tmp/lidar_height.png", y_fudge=Y_FUDGE)

    lidar_to_2d_front_view(lidar, v_res=VRES, h_res=HRES, v_fov=VFOV,
                           val="reflectance", saveto="/tmp/lidar_reflectance.png",
                           y_fudge=Y_FUDGE)


def load_file(filename):
    from array import array
    input_file = open(filename, 'r')
    float_array = array('f')
    float_array.fromstring(input_file.read())
    x = float_array[0::4]
    y = float_array[1::4]
    z = float_array[2::4]
    r = float_array[3::4]
    return (x, y, z, r)


def raw_plot():
    import matplotlib.pyplot as plt

    plt.figure(0)
    plt.clf()
    plt.plot(x, y, 'bx')
    plt.show()
    plt.figure(1)
    plt.clf()
    plt.plot(x, z, 'bx')
    plt.show()
    plt.figure(2)
    plt.clf()
    plt.plot(y, z, 'bx')
    plt.show()

    import numpy as np
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    n = 100
    ax.scatter(x[::100], y[::100], z[::100], c='b', marker='^')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

if __name__ == "__main__":
    import os

    vel_dir = '/Users/stanleybileschi/ResearchData/KITTI/raw-data/2011_09_26_drive_0001/velodyne_points/data'
    vel_dir = '/Users/ssi/Downloads/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/'
    vel_file = '0000000043.bin'
    vel_file = '0000000000.bin'
    lidar = _getLidarPoints(0, vel_dir)
    # (x, y, z, _) = load_file(os.path.join(vel_dir, vel_file))
    # lidar = load_file_points(os.path.join(vel_dir, vel_file))
    lidar_show(lidar)
