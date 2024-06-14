# encoding: utf-8
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist, squareform
from sklearn.manifold import MDS
from sklearn.preprocessing import StandardScaler
from nlink_distance.msg import DistanceArray_all
import rospy
from geometry_msgs.msg import PoseArray, Pose

new_data_uwb = np.zeros([4,4])

# mds_coordinates = np.zeros([4,2])
# 固定模块1和模块2的相对位置
fixed_points = np.array([[0, 0, 0], [1, 0, 1.2]])







# 使用MDS进行多维尺度分析，固定第一个和第二个点
class FixedMDS(MDS):
    def __init__(self, fixed_points, **kwargs):
        super().__init__(**kwargs)
        self.fixed_points = fixed_points



    def fit(self, X, y=None, init=None):
        X = np.asarray(X)
        n_samples = X.shape[0]

        # Initialize the embedding
        if init is None:
            init = self.random_state_.randn(n_samples, self.n_components)
        
        init[self.fixed_points[:, 0], :] = self.fixed_points[:, 1:]

        for i in range(10):  # Run for a fixed number of iterations
            dis = np.linalg.norm(init[:, np.newaxis, :] - init[np.newaxis, :, :], axis=-1)
            B = np.zeros_like(dis)
            mask = dis != 0
            B[mask] = -X[mask] / dis[mask]
            B[~mask] = 0
            B[np.diag_indices_from(B)] = -B.sum(axis=1)
            init -= 0.01 * B @ init
        
        self.embedding_ = init
        return self





def callback(msg):
        distance_matrix = []
        # print(new_data_uwb[0].shape)
        new_data_uwb[0] = np.array(msg.uwb0[:4:])
        # print(msg.uwb0[:3:])
        new_data_uwb[1] = np.array(msg.uwb1[:4:])
        new_data_uwb[2] = np.array(msg.uwb2[:4:])
        new_data_uwb[3] = np.array(msg.uwb3[:4:])
        
        # new_data_uwb[4] = np.array(msg.uwb4)
        # new_data_uwb[5] = np.array(msg.uwb5)
        # new_data_uwb[6] = np.array(msg.uwb6)
        # new_data_uwb[7] = np.array(msg.uwb7)
        if new_data_uwb.size == 4:
            distance_matrix = new_data_uwb
        distance_matrix = (new_data_uwb + new_data_uwb.T)/2
        print(distance_matrix)
        mds_coordinates = np.array(mds.fit_transform(distance_matrix))
        # print(mds_coordinates)
        
        data_pub(mds_coordinates)
            # self.index = (self.index + 1) % self.buffer_size


def data_pub(mds_coordinates):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"

        num_poses = 4
        for i in range(num_poses):
            pose = Pose()
            pose.position.x = mds_coordinates[i][0]
            pose.position.y = mds_coordinates[i][1]
            pose.position.z = 0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        pose_array_pub.publish(pose_array)

if __name__ == "__main__":
    # ros接受到的距离矩阵
    
    # 初始化ros节点
    rospy.init_node('pose_array_publisher', anonymous=True)
    #发布位置话题
    pose_array_pub = rospy.Publisher('/pose_array', PoseArray, queue_size=10)
    # 订阅距离话题
    distance_matrix_sub = rospy.Subscriber('/distance_topic_all',DistanceArray_all,callback) 
    mds = FixedMDS(fixed_points=fixed_points, n_components=2, dissimilarity='precomputed', random_state=42)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        # 使用FixedMDS
        # print(1)
        
        rate.sleep()







