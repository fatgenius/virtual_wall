#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <move_base_virtual_wall_server/CreateWall.h> 
#include "move_base_virtual_wall_server/WallInfo.h" 

#include <vector>
#include <std_msgs/Bool.h>
#include <sound_play/SoundRequest.h>
#include <sound_play/sound_play.h>
#include <sys/stat.h>

class CameraToMapTransformer {
public:
    CameraToMapTransformer() : nh_(), tf_listener_(),active_(false) {
        // 订阅相机生成的坐标点
        camera_point_sub_ = nh_.subscribe("/binocularcamera", 1, &CameraToMapTransformer::cameraPointCallback, this);

           nh_.param("point_distance_tolerance", tolerance_, 0.1f);
        // 订阅/cancel_command主题
        cancel_command_sub_ = nh_.subscribe("/cancel_command", 1, &CameraToMapTransformer::cancelCommandCallback, this);


        // 创建 ROS 服务客户端
        create_wall_client_ = nh_.serviceClient<move_base_virtual_wall_server::CreateWall>("/virtual_wall_server/create_wall");
        // 创建一个发布者向/binocularcamera话题发布消息
        binocularcamera_pub_ = nh_.advertise<std_msgs::Bool>("/binocularcamera", 1);

        sound_client_ = new sound_play::SoundClient();

        
    }


    ~CameraToMapTransformer() {
        delete sound_client_;
    }

    bool isDistanceSimilar(const std::vector<float>& points1, const std::vector<float>& points2, float tolerance) {
    if (points1.size() < 6 || points2.size() < 6) return false; // 确保有足够的数据点

    float distance1 = std::sqrt(std::pow(points2[0] - points1[0], 2) + std::pow(points2[1] - points1[1], 2) + std::pow(points2[2] - points1[2], 2));
    float distance2 = std::sqrt(std::pow(points2[3] - points1[3], 2) + std::pow(points2[4] - points1[4], 2) + std::pow(points2[5] - points1[5], 2));

    return std::fabs(distance1 - distance2) < tolerance;
}

    void cancelCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
        active_ = msg->data; // 如果/cancel_command为true，则停止处理；为false，则开始处理
        if (active_) {
            ROS_INFO("Camera to map transformer activated.");
        } else {
            ROS_INFO("Camera to map transformer deactivated.");
        }
    }

    void cameraPointCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

     if (!active_) {
            // 如果程序被设置为非活动状态，则不处理点
            return;
        }
    // 确保接收到的数据包含至少6个元素
    if (msg->data.size() < 6) {
        ROS_ERROR("Received data does not contain enough elements.");
        return;
    }
      if (!last_points_.empty() && isDistanceSimilar(last_points_, msg->data, 0.1)) {
       ROS_DEBUG("Received points are similar to the last points. Skipping processing.");
        return;
    }

    // 更新存储的点数据
    last_points_ = msg->data;

       // 使用data中前三个值作为起始点的x, y, z
    float x1 = msg->data[0];
    float y1 = msg->data[1];
    float z1 = msg->data[2];

    // 使用data中接下来的三个值作为结束点的x, y, z
    float x2 = msg->data[3];
    float y2 = msg->data[4];
    float z2 = msg->data[5];

    // 计算两点之间的欧氏距离
    float distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));

    // 检查距离是否在1米以内
    if (distance > 1.0) {
        // 如果两点距离超过1米，跳过处理
       ROS_DEBUG("The distance between the points is greater than 1 meter. Skipping processing.");
        return;
    }

    // 如果距离在1米以内，继续执行后续逻辑
    ROS_INFO("The distance between the points is within 1 meter. Proceeding with processing.");


    
    if (msg->data[6] == 0) {
        // 指定MP3文件的路径
        std::string mp3_file_path = "/home/nvidia/catkin_ws/src/virtual_wall/sounds/obs.mp3.mp3";

        // 检查文件是否存在
            if (!fileExists(mp3_file_path)) {
                ROS_ERROR("MP3 file does not exist: %s", mp3_file_path.c_str());
                return;
            }
        
        // 播放指定路径的MP3文件
        sound_client_->playWave(mp3_file_path);
        ROS_INFO("Playing MP3 file: %s", mp3_file_path.c_str());

        ros::Timer timer = nh_.createTimer(ros::Duration(3), [this](const ros::TimerEvent&) {
                std_msgs::Bool msg;
                msg.data = false;
                binocularcamera_pub_.publish(msg);
                ROS_INFO("Published false to /binocularcamera after 3 seconds.");
            }, true); // true表示定时器只执行一次
    }
    
    try {
        // 使用data中前三个值作为起始点的x, y, z
        geometry_msgs::PointStamped camera_point_start;
        camera_point_start.header.frame_id = "camera_link_optical"; // 相机坐标系的frame_id
        camera_point_start.header.stamp = ros::Time(0);
        camera_point_start.point.x = msg->data[0];
        camera_point_start.point.y = msg->data[1];
        camera_point_start.point.z = msg->data[2];

        // 使用data中接下来的三个值作为结束点的x, y, z
        geometry_msgs::PointStamped camera_point_end;
        camera_point_end.header.frame_id = "camera_link_optical";
        camera_point_end.header.stamp = ros::Time(0);
        camera_point_end.point.x = msg->data[3];
        camera_point_end.point.y = msg->data[4];
        camera_point_end.point.z = msg->data[5];

        // 转换起始点到地图坐标系
        geometry_msgs::PointStamped map_point_start;
        tf_listener_.transformPoint("map", camera_point_start, map_point_start);

        // 转换结束点到地图坐标系
        geometry_msgs::PointStamped map_point_end;
        tf_listener_.transformPoint("map", camera_point_end, map_point_end);

    //     // 调用服务发布转换后的点
    //     move_base_virtual_wall_server::CreateWall srv;
    //     srv.request.id = 0; // ID
    //     srv.request.start_point = map_point_start.point;
    //     srv.request.end_point = map_point_end.point; // 使用转换后的结束点

    //     if (create_wall_client_.call(srv)) {
    //         ROS_INFO("created successfully.");
    //     } else {
    //         ROS_ERROR("Failed to create wall.");
    //     }
    // } catch (tf::TransformException& ex) {
    //     ROS_ERROR("TransformException: %s", ex.what());
    // }

       // 构建WallInfo对象并填充数据
    move_base_virtual_wall_server::WallInfo wall;
    wall.id = 0; // 假设ID为0，根据需要调整
    wall.start_point = map_point_start.point;
    wall.end_point = map_point_end.point;

    // 调用服务发布转换后的点
    move_base_virtual_wall_server::CreateWall srv;
    srv.request.walls.push_back(wall); // 添加到请求的walls数组中

    if (create_wall_client_.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Wall created successfully.");
        } else {
            ROS_ERROR("Failed to create wall: Service executed, but reported failure.");
        }
    } else {
        ROS_ERROR("Failed to call create wall service.");
    }
} catch (tf::TransformException& ex) {
    ROS_ERROR("TransformException: %s", ex.what());
}
}

private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_point_sub_;
    ros::Subscriber cancel_command_sub_; 
    ros::Publisher binocularcamera_pub_; 
    ros::ServiceClient create_wall_client_;
    tf::TransformListener tf_listener_;
    bool active_;
    float tolerance_;
    sound_play::SoundClient* sound_client_;
    std::vector<float> last_points_;  

       bool fileExists(const std::string& path) {
        struct stat buffer;
        return (stat(path.c_str(), &buffer) == 0);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_to_map_transformer_node");
    CameraToMapTransformer transformer;
    ros::spin();
    return 0;
}



/**/

// #include <ros/ros.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <geometry_msgs/PointStamped.h>
// #include <tf/transform_listener.h>
// #include <move_base_virtual_wall_server/CreateWall.h>
// #include <std_msgs/Bool.h>
// #include <sound_play/sound_play.h>
// #include <sys/stat.h>

// class CameraToMapTransformer {
// public:
//     CameraToMapTransformer() : nh_(), tf_listener_(), active_(false) {
//         camera_point_sub_ = nh_.subscribe("/binocularcamera", 1, &CameraToMapTransformer::cameraPointCallback, this);
//         cancel_command_sub_ = nh_.subscribe("/cancel_command", 1, &CameraToMapTransformer::cancelCommandCallback, this);
//         create_wall_client_ = nh_.serviceClient<move_base_virtual_wall_server::CreateWall>("/virtual_wall_server/create_wall");
//         binocularcamera_pub_ = nh_.advertise<std_msgs::Bool>("/binocularcamera", 1);
//         sound_client_ = new sound_play::SoundClient();
//     }

//     ~CameraToMapTransformer() {
//         delete sound_client_;
//     }

//     void cancelCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
//         active_ = msg->data;
//         ROS_INFO(active_ ? "Camera to map transformer activated." : "Camera to map transformer deactivated.");
//     }

//     void cameraPointCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
//         if (!active_ || msg->data.size() < 6) {
//             ROS_ERROR("Operation halted due to inactive state or insufficient data elements.");
//             return;
//         }

//         if (msg->data[6] == 0) {
//             playSound("/home/nvidia/catkin_ws/src/virtual_wall/sounds/obs.mp3.mp3");
//         }

//         try {
//             auto [map_point_start, map_point_end] = transformPoints(msg->data);
//             publishTransformedPoints(map_point_start, map_point_end);
//         } catch (const tf::TransformException& ex) {
//             ROS_ERROR("TransformException: %s", ex.what());
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber camera_point_sub_, cancel_command_sub_; 
//     ros::Publisher binocularcamera_pub_; 
//     ros::ServiceClient create_wall_client_;
//     tf::TransformListener tf_listener_;
//     bool active_;
//     sound_play::SoundClient* sound_client_;

//     bool fileExists(const std::string& path) {
//         struct stat buffer;
//         return (stat(path.c_str(), &buffer) == 0);
//     }

//     void playSound(const std::string& mp3_file_path) {
//         if (!fileExists(mp3_file_path)) {
//             ROS_ERROR("MP3 file does not exist: %s", mp3_file_path.c_str());
//             return;
//         }
//         sound_client_->playWave(mp3_file_path);
//         ROS_INFO("Playing MP3 file: %s", mp3_file_path.c_str());
//         nh_.createTimer(ros::Duration(3), [this](const ros::TimerEvent&) {
//             std_msgs::Bool msg;
//             msg.data = false;
//             binocularcamera_pub_.publish(msg);
//             ROS_INFO("Published false to /binocularcamera after 3 seconds.");
//         }, true);
//     }

//     std::pair<geometry_msgs::PointStamped, geometry_msgs::PointStamped> transformPoints(const std::vector<float>& data) {
//         geometry_msgs::PointStamped camera_point_start, camera_point_end, map_point_start, map_point_end;
//         camera_point_start.header.frame_id = camera_point_end.header.frame_id = "camera_link_optical";
//         camera_point_start.header.stamp = camera_point_end.header.stamp = ros::Time(0);
//         camera_point_start.point.x = data[0]; camera_point_start.point.y = data[1]; camera_point_start.point.z = data[2];
//         camera_point_end.point.x = data[3]; camera_point_end.point.y = data[4]; camera_point_end.point.z = data[5];
//         tf_listener_.transformPoint("map", camera_point_start, map_point_start);
//         tf_listener_.transformPoint("map", camera_point_end, map_point_end);
//         return {map_point_start, map_point_end};
//     }

//     void publishTransformedPoints(const geometry_msgs::PointStamped& start, const geometry_msgs::PointStamped& end) {
//         move_base_virtual_wall_server::CreateWall srv;
//         srv.request.id = 0;
//         srv.request.start_point = start.point;
//         srv.request.end_point = end.point;
//         if (!create_wall_client_.call(srv)) {
//             ROS_ERROR("Failed to create wall.");
//         } else {
//             ROS_INFO("Wall created successfully.");
//         }
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "camera_to_map_transformer_node");
//     CameraToMapTransformer transformer;
//     ros::spin();
//     return 0;
// }