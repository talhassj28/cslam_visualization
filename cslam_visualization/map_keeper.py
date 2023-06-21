import os
import json
from cslam_visualization.srv import MapPath
from rclpy.node import Node

import open3d
import cslam.lidar_pr.icp_utils as icp_utils

class MapKeeper():

    def __init__(self, node, pose_graph_viz, pointcloud_viz):
        self.node = node
        self.pose_graph_viz = pose_graph_viz
        self.pointcloud_viz = pointcloud_viz
        self.srv = self.node.create_service(MapPath, "store_map", self.store_map_callback)
    
    def store_map_callback(self, request, response):
        self.node.get_logger().info("Incoming request: store map in path: " + request.path)
        response.path = request.path

        # TODO: check if / at the end of path
        # TODO: bug when internal folders no created
        # TODO: bug when file already exist
        
        robots_point_cloud = self.pointcloud_viz.pointclouds
        for robot_id in robots_point_cloud.keys():
            robot_folder = request.path + "/robot" + str(robot_id)
            os.mkdir(robot_folder)

            point_cloud_keyframes = robots_point_cloud[robot_id]
            for point_cloud_msg in point_cloud_keyframes:
                pcd_file_path = robot_folder + "/keyframe_" + str(point_cloud_msg.keyframe_id) + ".pcd"
                point_cloud = icp_utils.ros_to_open3d(point_cloud_msg.pointcloud)
                open3d.io.write_point_cloud(pcd_file_path, point_cloud)

        # TODO: change this implementation method, w rewrites the file everytime
        # TODO: Better error handling 
        # TODO: Allow passing file name as parameter
        try:
            pose_graph_path = request.path + "/pose_graph.json"
            with open(pose_graph_path, "w+") as json_file:
                # Initializa pose graph to be stored
                pose_graph_to_store = {}
                robot_poses_graphs = self.pose_graph_viz.robot_pose_graphs
                for robot_id in robot_poses_graphs.keys():
                    pose_graph_to_store[robot_id] = {
                        "edges": {},
                        "values": {}
                    }

                    # Store pose graph values (nodes)
                    keyframes = robot_poses_graphs[robot_id]
                    for keyframe in keyframes.keys():
                        pose = keyframes[keyframe].pose
                        pose_graph_to_store[robot_id]["values"][keyframe] = {
                            "position": {
                                "x": pose.position.x,
                                "y": pose.position.y,
                                "z": pose.position.z
                            },
                            "orientation": {
                                "x": pose.orientation.x,
                                "y": pose.orientation.y,
                                "z": pose.orientation.z,
                                "w": pose.orientation.w
                            }
                        }

                    # Store pose graph edges
                    robot_edges = self.pose_graph_viz.robot_pose_graphs_edges[robot_id]     
                    pose_graph_to_store[robot_id]["edges"] = []
                    for edge in robot_edges:
                        pose_graph_to_store[robot_id]["edges"].append({
                        "key_from": {
                            "robot_id": edge.key_from.robot_id,
                            "keyframe_id": edge.key_from.keyframe_id
                        },
                        "key_to": {
                            "robot_id": edge.key_to.robot_id,
                            "keyframe_id": edge.key_to.keyframe_id
                        },
                        "measurement": {
                            "position": {
                                "x": edge.measurement.position.x,
                                "y": edge.measurement.position.y,
                                "z": edge.measurement.position.z
                            },
                            "orientation": {
                                "x": edge.measurement.orientation.x,
                                "y": edge.measurement.orientation.y,
                                "z": edge.measurement.orientation.z,
                                "w": edge.measurement.orientation.w,
                            },
                        },
                        "noise_std": edge.noise_std.tolist()
                    })  

                json.dump(pose_graph_to_store, json_file)
        except:
            self.node.get_logger().info(f"Error: File '{request.path}' not found.")
        
        return response
