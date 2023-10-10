import numpy as np

# import rosbag

# from rospy_message_converter import message_converter

from sensor_msgs.msg import PointCloud2
from tritech_micron.msg import TritechMicronConfig

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def pointcloud_to_image(pc_msg: PointCloud2, interpolate=True, resolution=0.5, cmap=None):
    """
    Converts the 2D pointcloud to image. Each image pixel comes from interpolation
    """

    # PointCloud2 to numpy

    pc_np = np.reshape(np.frombuffer(pc_msg.data, dtype=np.float32), (-1,4))

    # min/max

    xy_min = np.min(pc_np[:, :2])
    xy_max = np.max(pc_np[:, :2])

    x = pc_np[:, 0]
    y = pc_np[:, 1]
    z = pc_np[:, 3]

    if not interpolate:
        return (x, y, z)

    # np.array

    import scipy.interpolate
    from scipy.interpolate import LinearNDInterpolator, CloughTocher2DInterpolator, NearestNDInterpolator

    # import matplotlib.pyplot as plt

    # z_all = plotGrid.astype(float)            # convert nones to nan
    # x_all, y_all = np.indices(plotGrid.shape) # get x and y coordinates

    print(f"min(x), max(x), min(y), max(y): {np.min(x)}, {np.max(x)}, {np.min(y)}, {np.max(y)}")

    X = np.linspace(min(x), max(x), num=500)
    Y = np.linspace(min(y), max(y), num=500)
    X, Y = np.meshgrid(X, Y)
    interp = LinearNDInterpolator(list(zip(x, y)), z)
    # interp = NearestNDInterpolator(list(zip(x, y)), z)
    # interp = CloughTocher2DInterpolator(list(zip(x, y)), z)        
    Z = interp(X, Y)

    # plt.scatter(x, y, c=z, s=0.5)
    # plt.pcolormesh(X, Y, Z, shading='auto')

    # # plt.plot(x, y, "ok", label="input point")
    # # plt.legend()

    # plt.colorbar()

    # plt.axis("equal")

    # plt.show()

    # x_all = np.arange(xy_min[0], xy_max[0], resolution)
    # y_all = np.arange(xy_min[1], xy_max[1], resolution)

    # # convert to 1d arrays of coordinates
    # valid = ~np.isnan(z_all)
    # x, y, z = x_all[valid], y_all[valid], z_all[valid]

    # # interpolate
    # # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp2d.html
    # # jax: https://github.com/google/jax/discussions/10689
    # interp = scipy.interpolate.interp2d(x, y, z)
    # filled_data = interp(x_all[:,0], y_all[0,:])  # this is kinda gross, but `interp` doesn't
    #                                             # do normal broadcasting

    # return filled_data

    return (X, Y, Z)

# def add_row(img_data_1, img_data_2, config):


# def main():

#     # bag_path = '/media/miron/ubuntu_20/uuv_ws/2023-04-28-13-32-01.bag'
#     # bag_path = '/media/miron/Elements/sonar_tests/2023-05-10-11-33-43.bag'
#     bag_path = '/media/miron/Elements/sonar_tests/2023-05-10-13-58-37.bag'

#     cfg_default = {
#         'inverted': True,
#         'continuous': True,
#         'scanright': True,
#         'adc8on': True,
#         'gain': 0.5,
#         'ad_low': 0.0,
#         'ad_high': 80.0,
#         'left_limit': 2.356194490192345,
#         'right_limit': 3.926009069282995,
#         'range': 10.0,
#         'nbins': 399,
#         'step': 0.031415926535897934
#     }

#     cfg_keys = ['ad_low', 'ad_high', 'gain', 'left_limit', 'nbins', 'range', 'right_limit', 'speed', 'step']

#     bag_open = rosbag.Bag(bag_path, 'r')

#     new_cloud = False
#     new_config = False

#     cloud_id = 0

#     row_id = 0
#     max_rows = 4

#     # https://matplotlib.org/stable/gallery/misc/multipage_pdf.html
#     # https://matplotlib.org/stable/gallery/subplots_axes_and_figures/subplots_demo.html
#     with PdfPages('multipage_pdf.pdf') as pdf:        

#         for topic, msg, t in bag_open.read_messages():
#             if topic == "/merged_cloud":
#                 # if cloud_id<=1 or cloud_id>=55:
#                 if cloud_id<=1 or cloud_id>=120:    
#                     new_cloud = False
#                 else:
#                     x_1, y_1, z_1 = pointcloud_to_image(msg, interpolate=False)
#                     x_2, y_2, z_2 = pointcloud_to_image(msg)
#                     print("found")
#                     new_cloud = True

#                 cloud_id += 1
#                 # break
#             elif topic == "/merged_config":
#                 config = message_converter.convert_ros_message_to_dictionary(msg)
#                 config.pop("header")
#                 print(f"config: {config}")
#                 new_config = True
#             if new_cloud and new_config:
#                 if row_id == 0:
#                     # new page
#                     fig, axs = plt.subplots(max_rows, 2, figsize=(20, 40))
#                     # plt.figure(figsize=(30, 20))

#                 # add image and description
#                 # add_row(img_1, img_2, config)

#                 cfg_title = {k:config[k] for k in cfg_keys if k in config}

#                 # axs[row_id, 0].set_title(cfg_title)
#                 axs[row_id, 0].set_title(cfg_title, loc='left')
#                 axs[row_id, 0].scatter(x_1, y_1, c=z_1, s=0.5, rasterized=True)
#                 axs[row_id, 0].axis("equal")
#                 axs[row_id, 1].pcolormesh(x_2, y_2, z_2, shading='auto', rasterized=True)
#                 axs[row_id, 1].axis("equal")

#                 row_id = (row_id + 1) % max_rows

#                 if row_id == 0:
#                     # close page
#                     pdf.savefig()  # saves the current figure into a pdf page
#                     plt.close()

#                 new_cloud = False
#                 new_config = False

# def test():

#     # bag_path = '/media/miron/ubuntu_20/uuv_ws/2023-04-28-13-32-01.bag'
#     # bag_path = '/media/miron/Elements/sonar_tests/2023-05-10-11-33-43.bag'
#     bag_path = '/media/miron/Elements/sonar_tests/2023-05-10-13-58-37.bag'

#     cfg_default = {
#         'inverted': True,
#         'continuous': True,
#         'scanright': True,
#         'adc8on': True,
#         'gain': 0.5,
#         'ad_low': 0.0,
#         'ad_high': 80.0,
#         'left_limit': 2.356194490192345,
#         'right_limit': 3.926009069282995,
#         'range': 10.0,
#         'nbins': 399,
#         'step': 0.031415926535897934
#     }

#     cfg_keys = ['ad_low', 'ad_high', 'gain', 'left_limit', 'nbins', 'range', 'right_limit', 'step']

#     bag_open = rosbag.Bag(bag_path, 'r')

#     new_cloud = False
#     new_config = False

#     cloud_id = 0

#     row_id = 0
#     max_rows = 4   

#     for topic, msg, t in bag_open.read_messages():
#         if topic == "/merged_cloud":
#             if cloud_id<=1 or cloud_id>=55:
#                 new_cloud = False
#             else:
#                 x_1, y_1, z_1 = pointcloud_to_image(msg, interpolate=False)
#                 x_2, y_2, z_2 = pointcloud_to_image(msg)
#                 print("found")
                
#                 fig, axs = plt.subplots(2, figsize=(10, 20))
#                 axs[0].scatter(x_1, y_1, c=z_1, s=0.5)
#                 axs[0].axis("equal")
#                 axs[1].pcolormesh(x_2, y_2, z_2, shading='auto')
#                 axs[1].axis("equal")

#                 plt.show()
#                 break

#             cloud_id += 1
                


# if __name__ == "__main__":

#     main()
#     # test()