import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import math

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('simple_filter')
        
        # 订阅 Fast-LIO 的 Body Frame 话题
        # 务必确认你的话题名是 /cloud_registered_body 或 /CloudBody
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_registered_body', 
            self.listener_callback,
            10)
            
        self.publisher = self.create_publisher(PointCloud2, '/cloud_clamped', 10)
        
        # 设定阈值：5米的平方是25
        self.MAX_DISTANCE_SQ = 5.0 ** 2
        
        self.get_logger().info('高性能 NumPy 过滤器已启动 (C-contiguous 修复版)')

    def listener_callback(self, msg):
        try:
            # 1. 转换数据为 numpy uint8 数组 (原始字节流)
            # data 是 bytes, 需要转成 uint8 的 numpy 数组才能操作
            raw_data = np.frombuffer(msg.data, dtype=np.uint8)
            
            # 2. Reshape 为 (N, point_step) 的矩阵
            # 这样每一行就是一个点的所有字节数据
            points_data = raw_data.reshape(-1, msg.point_step)
            
            # 3. 提取 X, Y, Z (修复报错的关键步骤)
            # 偏移量通常是 x=0, y=4, z=8 (对于 float32)
            # [ERROR FIX]: 加上 .copy() 强制内存连续，解决 "must be C-contiguous" 错误
            x = points_data[:, 0:4].copy().view(np.float32).flatten()
            y = points_data[:, 4:8].copy().view(np.float32).flatten()
            z = points_data[:, 8:12].copy().view(np.float32).flatten()
            
            # 4. 向量化计算距离平方 (极速)
            dist_sq = x**2 + y**2 + z**2
            
            # 5. 生成掩码 (Mask): 找出所有距离小于 5米的行
            mask = dist_sq < self.MAX_DISTANCE_SQ
            
            # 6. 利用掩码直接筛选原始字节数据
            # points_data 是 uint8 类型，我们只保留 mask 为 True 的行
            filtered_raw = points_data[mask]
            
            # 7. 转换回 bytes 准备发布
            filtered_bytes = filtered_raw.tobytes()
            
            # 8. 重新打包 PointCloud2
            new_msg = PointCloud2()
            new_msg.header = msg.header
            new_msg.height = 1
            new_msg.width = filtered_raw.shape[0] # 剩余点的数量
            new_msg.fields = msg.fields
            new_msg.is_bigendian = msg.is_bigendian
            new_msg.point_step = msg.point_step
            new_msg.row_step = new_msg.width * msg.point_step
            new_msg.is_dense = msg.is_dense
            new_msg.data = filtered_bytes
            
            self.publisher.publish(new_msg)
            
        except Exception as e:
            # 打印详细错误，防止再次静默失败
            self.get_logger().error(f'处理出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()