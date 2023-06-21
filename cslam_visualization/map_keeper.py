import os
import json
from cslam_visualization.srv import MapPath
from rclpy.node import Node

class MapKeeper():

    def __init__(self, node, pose_graph_viz, pointcloud_viz):
        self.node = node
        self.pose_graph_viz = pose_graph_viz
        self.pointcloud_viz = pointcloud_viz
        self.srv = self.node.create_service(MapPath, "store_map", self.store_map_callback)

    # def store_map_callback(self, request, response):
    #     # response.sum = request.a + request.b + request.c
    #     response.path = "hello world"
    #     self.node.get_logger().info("Incoming request: store map in path: " + request.path)

    #     return response
    
    def store_map_callback(self, request, response):
        response.path = request.path
        self.node.get_logger().info("Incoming request: store map in path: " + request.path)

        # TODO: path should be a param 
        file_path = "/home/romantwice/Projects/MISTLab/Swarm-SLAM/src/cslam_visualization/pose_graph.json"
        
        # TODO: change this implementation method, w rewrites the file everytime
        try:
            with open(request.path, "w+") as json_file:
                pose_graph_to_store = {}
                # if (os.path.exists(request.path)):
                #     try:
                #         pose_graph_to_store = json.load(json_file)
                #     except:
                #         self.node.get_logger().info("File empty")
                
                robot_poses_graphs = self.pose_graph_viz.robot_pose_graphs
                for robot_id in robot_poses_graphs.keys():
                    pose_graph_to_store[robot_id] = {
                        "edges": {},
                        "values": {}
                    }

                    keyframes = robot_poses_graphs[robot_id].keys()
                    for keyframe in keyframes:
                        pose = keyframes[keyframe]
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
                            

                # for pose_graph_value in msg.values:
                #     pose_graph_to_store[msg.robot_id]["values"][pose_graph_value.key.keyframe_id] = {
                #         "position": {
                #             "x": pose_graph_value.pose.position.x,
                #             "y": pose_graph_value.pose.position.y,
                #             "z": pose_graph_value.pose.position.z
                #         },
                #         "orientation": {
                #             "x": pose_graph_value.pose.orientation.x,
                #             "y": pose_graph_value.pose.orientation.y,
                #             "z": pose_graph_value.pose.orientation.z,
                #             "w": pose_graph_value.pose.orientation.w
                #         }
                #     }
                    
                # for pose_graph_edge in msg.edges:
                #     pose_graph_to_store[msg.robot_id]["edges"][pose_graph_value.key.keyframe_id] = {
                #         "key_from": {
                #             "robot_id": pose_graph_edge.key_from.robot_id,
                #             "keyframe_id": pose_graph_edge.key_from.keyframe_id
                #         },
                #         "key_to": {
                #             "robot_id": pose_graph_edge.key_to.robot_id,
                #             "keyframe_id": pose_graph_edge.key_to.keyframe_id
                #         },
                #         "measurement": {
                #             "position": {
                #                 "x": pose_graph_edge.measurement.position.x,
                #                 "y": pose_graph_edge.measurement.position.y,
                #                 "z": pose_graph_edge.measurement.position.z
                #             },
                #             "orientation": {
                #                 "x": pose_graph_edge.measurement.orientation.x,
                #                 "y": pose_graph_edge.measurement.orientation.y,
                #                 "z": pose_graph_edge.measurement.orientation.z,
                #                 "w": pose_graph_edge.measurement.orientation.w,
                #             },
                #         },
                #         "noise_std": pose_graph_edge.noise_std.tolist()
                #     }
                    # self.node.get_logger().info(str(type(pose_graph_edge.noise_std)))
                    
                # Write the data to the JSON file
                json.dump(pose_graph_to_store, json_file)
        except:
            self.node.get_logger().info(f"Error: File '{request.path}' not found.")
        
        return response
