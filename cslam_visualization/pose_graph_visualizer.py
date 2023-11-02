import rclpy
import json
import os.path
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
from visualization_msgs.msg import MarkerArray, Marker
from distinctipy import distinctipy

class PoseGraphVisualizer():

    def __init__(self, node, params):
        self.node = node
        self.params = params
        self.pose_graph_markers_publisher = self.node.create_publisher(
            MarkerArray, "/cslam/viz/pose_graph_markers", 10)
        self.nb_colors = self.params["nb_colors"]
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]  
        self.colors = distinctipy.get_colors(self.nb_colors)
        self.pose_graph_subscriber = self.node.create_subscription(
            PoseGraph, '/cslam/viz/pose_graph', self.pose_graph_callback, 10)
        self.robot_pose_graphs = {}
        self.robot_pose_graphs_edges = {}
        self.origin_robot_ids = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)
        
    def store_pose_graph(self, msg):
        # TODO: path should be a param 
        file_path = "/home/romantwice/Projects/MISTLab/Swarm-SLAM/src/cslam_visualization/pose_graph.json"
        
        # TODO: change this implementation method, w rewrites the file everytime
        try:
            with open(file_path, "w+") as json_file:
                pose_graph_to_store = {}
                if (os.path.exists(file_path)):
                    try:
                        pose_graph_to_store = json.load(json_file)
                    except:
                        self.node.get_logger().info("File empty")
                            
                if msg.robot_id not in pose_graph_to_store:
                    pose_graph_to_store[msg.robot_id] = {
                        "edges": {},
                        "values": {}
                    }

                for pose_graph_value in msg.values:
                    pose_graph_to_store[msg.robot_id]["values"][pose_graph_value.key.keyframe_id] = {
                        "position": {
                            "x": pose_graph_value.pose.position.x,
                            "y": pose_graph_value.pose.position.y,
                            "z": pose_graph_value.pose.position.z
                        },
                        "orientation": {
                            "x": pose_graph_value.pose.orientation.x,
                            "y": pose_graph_value.pose.orientation.y,
                            "z": pose_graph_value.pose.orientation.z,
                            "w": pose_graph_value.pose.orientation.w
                        }
                    }
                    
                for pose_graph_edge in msg.edges:
                    pose_graph_to_store[msg.robot_id]["edges"][pose_graph_value.key.keyframe_id] = {
                        "key_from": {
                            "robot_id": pose_graph_edge.key_from.robot_id,
                            "keyframe_id": pose_graph_edge.key_from.keyframe_id
                        },
                        "key_to": {
                            "robot_id": pose_graph_edge.key_to.robot_id,
                            "keyframe_id": pose_graph_edge.key_to.keyframe_id
                        },
                        "measurement": {
                            "position": {
                                "x": pose_graph_edge.measurement.position.x,
                                "y": pose_graph_edge.measurement.position.y,
                                "z": pose_graph_edge.measurement.position.z
                            },
                            "orientation": {
                                "x": pose_graph_edge.measurement.orientation.x,
                                "y": pose_graph_edge.measurement.orientation.y,
                                "z": pose_graph_edge.measurement.orientation.z,
                                "w": pose_graph_edge.measurement.orientation.w,
                            },
                        },
                        "noise_std": pose_graph_edge.noise_std.tolist()
                    }
                    # self.node.get_logger().info(str(type(pose_graph_edge.noise_std)))
                    
                # Write the data to the JSON file
                self.node.get_logger().info("INSIDE JUMP")
                json.dump(pose_graph_to_store, json_file)
        except:
            self.node.get_logger().info(f"Error: File '{file_path}' not found.")

    def pose_graph_callback(self, msg):
        self.origin_robot_ids[msg.robot_id] = msg.origin_robot_id
        if msg.robot_id not in self.robot_pose_graphs:
            self.robot_pose_graphs[msg.robot_id] = {}

        for pose in msg.values:
            self.robot_pose_graphs[msg.robot_id][pose.key.keyframe_id] = pose
            
        self.robot_pose_graphs_edges[msg.robot_id] = msg.edges
        # TODO: remove
        # self.store_pose_graph(msg)

    def robot_pose_graphs_to_marker_array(self):
        """Converts a PoseGraph messages to a MarkerArray message"""
        marker_array = MarkerArray()

        # Nodes (poses)
        for robot_id, pose_graph in self.robot_pose_graphs.items():
            color = self.colors[robot_id % self.nb_colors]
            marker = Marker()
            marker.header.frame_id = "robot" + str(self.origin_robot_ids[robot_id]) + "_map"
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = "poses"
            marker.id = robot_id
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.params["pose_graph_markers_size"]
            marker.scale.y = self.params["pose_graph_markers_size"]
            marker.scale.z = self.params["pose_graph_markers_size"]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.frame_locked = True
            for key, node in pose_graph.items():
                marker.points.append(node.pose.position)
            marker_array.markers.append(marker)

        # Edges (constraints)
        for robot_id, pose_graph in self.robot_pose_graphs.items():
            color = self.colors[robot_id % self.nb_colors]
            marker = Marker()
            marker.header.frame_id = "robot" + str(self.origin_robot_ids[robot_id]) + "_map"
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = "edges"
            marker.id = robot_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.params["pose_graph_markers_size"] / 2
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.frame_locked = False
            for edge in self.robot_pose_graphs_edges[robot_id]:
                if edge.key_from.robot_id in self.robot_pose_graphs and edge.key_to.robot_id in self.robot_pose_graphs:
                    if edge.key_from.keyframe_id in self.robot_pose_graphs[edge.key_from.robot_id] and edge.key_to.keyframe_id in self.robot_pose_graphs[edge.key_to.robot_id]:
                        marker.points.append(
                            self.robot_pose_graphs[edge.key_from.robot_id][edge.key_from.keyframe_id].pose.position)
                        marker.points.append(
                            self.robot_pose_graphs[edge.key_to.robot_id][edge.key_to.keyframe_id].pose.position)
            marker_array.markers.append(marker)

        return marker_array
 
    def visualization_callback(self):
        marker_array = self.robot_pose_graphs_to_marker_array()
        self.pose_graph_markers_publisher.publish(marker_array)
