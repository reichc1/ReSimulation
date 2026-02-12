import xml.etree.ElementTree as ET
import math
import numpy as np
import pandas as pd
import os
import re
import json
from scenariogeneration import xosc, Scenario
from LaneCoordinateTransformationfromGPS import gps_to_utm, gps_to_lane_coordinates, world_to_lane_coordinates


class Vehicle:
    """Vehicle class with basic attributes and methods.
    
    Attributes:
        name: Name of the vehicle.
        data: Data of the vehicle.
        end_time: End time of the vehicle.
    """

    def __init__(self, name, input_data) -> None:
        """Initialization of a vehicle.

        Args:
            name: Name of the vehicle.
            input_data: Data of the vehicle.
        """
        self.__name = name
        self.__data = input_data
        self.__end_time = input_data.index.max()

    def getName(self):
        """Returns the name of the vehicle."""
        return self.__name

    def getEndTime(self):
        """Returns the end time of the vehicle."""
        return self.__end_time
    
    def getData(self):
        """Returns the data of the vehicle."""
        return self.__data


class GenerateScenario(object):

    def __init__(self) -> None:
        self.__vehicles = []

    def setVehicles(self, vehicles):
        self.__vehicles = vehicles

    def _createOscVehicle(self, vehicle):
        if vehicle.getName() == "Ego":
            OscVehicle = xosc.CatalogReference('VehicleCatalog', 'car_red')
        else:
            OscVehicle = xosc.CatalogReference('VehicleCatalog', 'car_white')
        return OscVehicle


    # Parsing the OpenDRIVE file to extract road, geometry, and lane data
    def parse_opendrive(self, file_path):
        tree = ET.parse(file_path)
        root = tree.getroot()
        roads = {}
        road_links = {}
        
        # First, parse all roads and store their details
        for road in root.findall('road'):
            road_id = int(road.get('id'))
            road_length = float(road.get('length'))
            junction = road.get('junction')
            
            # Extract geometries
            geometries = []
            for geometry in road.find('planView').findall('geometry'):
                s = float(geometry.get('s'))
                x = float(geometry.get('x'))
                y = float(geometry.get('y'))
                hdg = float(geometry.get('hdg'))
                segment_length = float(geometry.get('length'))
                
                geo_type = list(geometry)[0].tag if list(geometry) else 'unknown'
                geo_data = geometry.find(geo_type).attrib if geo_type != 'line' else {}
                
                geometries.append({
                    's': s,
                    'x': x,
                    'y': y,
                    'hdg': hdg,
                    'length': segment_length,
                    'type': geo_type,
                    'data': geo_data
                })
            
            # Extract lane sections
            lane_sections = []
            lanes_element = road.find('lanes')
            if lanes_element is not None:
                for lane_section in lanes_element.findall('laneSection'):
                    lane_s = float(lane_section.get('s'))
                    lanes = []
                    
                    for lane in lane_section.findall('lane'):
                        lane_id = int(lane.get('id'))
                        widths = []
                        for width in lane.findall('width'):
                            width_s_offset = float(width.get('sOffset'))
                            a, b, c, d = map(float, [width.get('a'), width.get('b'), width.get('c'), width.get('d')])
                            widths.append({
                                'sOffset': width_s_offset,
                                'a': a, 'b': b, 'c': c, 'd': d
                            })
                        lanes.append({'id': lane_id, 'widths': widths})
                    
                    lane_sections.append({'s': lane_s, 'lanes': lanes})
            
            # Extract predecessor and successor links
            link = road.find('link')
            predecessor = None
            successor = None
            
            if link is not None:
                pred_elem = link.find('predecessor')
                succ_elem = link.find('successor')
                if pred_elem is not None and pred_elem.get('elementType') == 'road':
                    predecessor = int(pred_elem.get('elementId'))
                if succ_elem is not None and succ_elem.get('elementType') == 'road':
                    successor = int(succ_elem.get('elementId'))
            
            roads[road_id] = {
                'id': road_id,
                'length': road_length,
                'junction': junction,
                'predecessor': predecessor,
                'successor': successor,
                'geometries': geometries,
                'laneSections': lane_sections
            }
            
        return roads

    def parse_odr_for_road_order(self, roads, file_path, actual_lane_id):
        tree = ET.parse(file_path)
        root = tree.getroot()
        junctions = {}
        ordered_roads = []
        
        # Parse junctions
        for junction in root.findall('junction'):
            junction_id = junction.get('id')
            connections = []
            for connection in junction.findall('connection'):
                incoming_road = connection.get('incomingRoad')
                connecting_road = connection.get('connectingRoad')
                contact_point = connection.get('contactPoint')
                lane_links = connection.findall('laneLink')
                
                # Only keep lane links matching actual_lane_id
                for lane_link in lane_links:
                    from_lane = int(lane_link.get('from'))
                    to_lane = int(lane_link.get('to'))
                    if from_lane == -actual_lane_id and to_lane == -actual_lane_id:
                        connections.append((incoming_road, connecting_road, contact_point))
                        break
            junctions[junction_id] = connections
        
        # Order roads correctly
        for road_id, road in roads.items():
            if road['junction'] == '-1':
                ordered_roads.append(road)
            else:
                junction_id = road['junction']
                if junction_id in junctions:
                    for incoming_road, connecting_road, contact_point in junctions[junction_id]:
                        if str(road_id) == connecting_road:
                            index = next((i for i, r in enumerate(ordered_roads) if str(r['id']) == incoming_road), -1)
                            if index != -1:
                                ordered_roads.insert(index + 1, road)
                                break
        
        return ordered_roads


    # Generate Trajectory while considering Road_id-Changes
    def generate_trajectory(self, df_points, df_ego, initial_s, initial_lane_id, initial_offset, road_id, roads, vehicle_name):
        trajectory_name = f"PolylineTrajectory_{vehicle_name}"
        trajectory = xosc.Trajectory(trajectory_name, closed=False)

        list_points = []
        current_s = initial_s
        current_lane_id = initial_lane_id
        current_offset = initial_offset

        # Compute initial distances
        matching_road = next((road for road in roads if road['id'] == int(road_id)), None)
        if matching_road is None:
            raise ValueError(f"Road ID {road_id} not found in roads: {[road['id'] for road in roads]}")

        remaining_road_id_distance = matching_road['length'] - initial_s
        remaining_total_trajectory_distance = df_points["delta_s"].sum()

        # Get lane sections of the current road
        current_road_index = next(i for i, road in enumerate(roads) if road['id'] == road_id)
        lane_sections = roads[current_road_index]['laneSections']
        current_lane_section_index = 0
        current_lane_section_length = lane_sections[current_lane_section_index]['s']

        for index, row in df_points.iterrows():
            if row["delta_s"] is None or math.isnan(row["delta_s"]):
                continue  # Skip invalid data points
            delta_s = row["delta_s"]

            # Update remaining distances
            remaining_road_id_distance -= delta_s
            remaining_total_trajectory_distance -= delta_s

            # Check if we need to switch roads
            if remaining_road_id_distance <= 0:
                # Move to the next road
                current_road_index += 1
                if current_road_index >= len(roads):
                    break  # No more roads left
                
                road_id = roads[current_road_index]['id']
                road_length = roads[current_road_index]['length']
                
                # Reinitialize remaining_road_id_distance
                remaining_road_id_distance = road_length - delta_s
                
                # Reset s-coordinate when switching roads
                current_s = 0
                # Update lane sections
                lane_sections = roads[current_road_index]['laneSections']

            # Update current_s and lane attributes
            current_s += delta_s
            current_lane_id = -(int(row["lane_id"]))  # Negate to match lane_id format
            current_offset = row["t_coordinate"]

            # Create lane position
            lane_point = xosc.LanePosition(
                s=current_s,
                lane_id=current_lane_id,
                offset=current_offset,
                road_id=road_id
            )
            list_points.append(lane_point)


        # Create the Polyline Shape for the trajectory
        # Create the time_values list
        df_points = df_points.set_index("timestamp")
        time_values = df_points.index.values.tolist()
        polyline = xosc.Polyline(time=time_values, positions=list_points)
        trajectory.add_shape(polyline)

        return trajectory  

    def create_trajectory_catalog(self, output_folder, road_network_file, abstraction_path_output_dir, username):
      catalog = xosc.CatalogFile()
      scenario_output_dir = os.path.join(output_folder)
      os.makedirs(scenario_output_dir, exist_ok=True)
      save_path = os.path.join(scenario_output_dir, "TrajectoryCatalog.xosc")
      catalog.create_catalog(save_path, "TrajectoryCatalog", username, "Todo Description")

      # Ego trajectory
      print("✅Generate trajectory for EGO")
      parquet_file_path = os.path.join(abstraction_path_output_dir, "Ego_ReSimulationInput.parquet")
      df_ego = pd.read_parquet(parquet_file_path)

      # Load OpenDRIVE data
      unordered_roads = self.parse_opendrive(road_network_file)
      roads = self.parse_odr_for_road_order(unordered_roads, road_network_file, 1)

      # Get GPS data for initialization
      gps_info = df_ego.iloc[0]["initial_GPS_position"]
      lat = float(gps_info["dr_lat"])
      lon = float(gps_info["dr_long"])
      azimuth_ego = float(gps_info["azimuth"])

      x_ego, y_ego, h_ego = gps_to_utm(lat, lon, azimuth_ego)
      s, t, r = world_to_lane_coordinates(x_ego, y_ego, h_ego, road_network_file)
      r = int(r)

      # Initial t-coordinate and lane_id
      t = df_ego.iloc[0]["t_coordinate"]
      initial_lane_id = -int(df_ego.iloc[0]["lane_id"])

      trajectory = self.generate_trajectory(df_ego, df_ego, initial_s=s, initial_lane_id=initial_lane_id, initial_offset=t, road_id=r, roads=roads, vehicle_name="Ego")
      catalog.add_to_catalog(trajectory)

      # Other vehicles
      for vehicle in self.__vehicles:
          if vehicle.getName() != "Ego":
              print("✅Generate trajectory for ", vehicle.getName())
              match = re.search(r'([0-9]+[a-zA-Z]*)', vehicle.getName())
              parquet_file_path = os.path.join(abstraction_path_output_dir, f"{match.group(1)}_ReSimulationInput.parquet")
              df = pd.read_parquet(parquet_file_path)
              first_time_value = df.iloc[0]["timestamp"]

              # Load OpenDRIVE data again
              unordered_roads = self.parse_opendrive(road_network_file)
              roads = self.parse_odr_for_road_order(unordered_roads, road_network_file, 1)

              # Get GPS data from the vehicle's input parquet
              gps_info = df.iloc[0]["initial_GPS_position"]
              lat = float(gps_info["dr_lat"])
              lon = float(gps_info["dr_long"])
              azimuth = azimuth_ego  # still use ego azimuth

              x, y, h = gps_to_utm(lat, lon, azimuth)
              if math.copysign(1, h) != math.copysign(1, h_ego):
                  h = -h

              s, t, r = world_to_lane_coordinates(x, y, h, road_network_file)
              r = int(r)

              t = df.iloc[0]["t_coordinate"]
              initial_lane_id = -int(df.iloc[0]["lane_id"])

              trajectory = self.generate_trajectory(df, df_ego, initial_s=s, initial_lane_id=initial_lane_id, initial_offset=t, road_id=r, roads=roads, vehicle_name=vehicle.getName())
              catalog.add_to_catalog(trajectory)

      catalog.dump()
      return save_path


    def _createAddEntityAction(self, vehicle, first_time_value):
        """Creates an add entity action for the vehicle at its first time_value."""
        
        # Define a position where the vehicle should be added
        trajectory = xosc.CatalogReference('TrajectoryCatalog', "PolylineTrajectory_" + vehicle.getName())
        position = xosc.TrajectoryPosition(trajectory, 0.0)

        # Trigger when simulation time reaches the first timestamp of the vehicle
        trigcond = xosc.SimulationTimeCondition(first_time_value, xosc.Rule.greaterOrEqual)
        trigger = xosc.ValueTrigger(f"Add{vehicle.getName()}Trigger", 0, xosc.ConditionEdge.rising, trigcond)

        # Add entity action
        add_action = xosc.AddEntityAction(vehicle.getName(), position) 

        # Create event
        event_add = xosc.Event(f"Add{vehicle.getName()}Event", xosc.Priority.parallel)
        event_add.add_trigger(trigger)
        event_add.add_action(f"Add{vehicle.getName()}Action", add_action)

        # Create maneuver and maneuver group
        man = xosc.Maneuver(f"Add{vehicle.getName()}Maneuver")
        man.add_event(event_add)

        mangr = xosc.ManeuverGroup(f"Add{vehicle.getName()}ManeuverGroup")
        mangr.add_actor(vehicle.getName())
        mangr.add_maneuver(man)

        # Start trigger (based on simulation time)
        starttrigger = xosc.ValueTrigger(f"StartAdd{vehicle.getName()}Condition", 0, xosc.ConditionEdge.rising, xosc.SimulationTimeCondition(0, xosc.Rule.greaterThan))
        act = xosc.Act(f"Add{vehicle.getName()}Act", starttrigger)
        act.add_maneuver_group(mangr)

        return act


    def _createDeleteEntityAction(self, vehicle, data_transformation_path_output_dir):
        """Creates a delete entity action for the vehicle."""

        # Load simulation data from the specified Parquet file to access timing info
        parquet_file_path = os.path.join(data_transformation_path_output_dir, "Ego_ReSimulationInput.parquet")
        df_ego = pd.read_parquet(parquet_file_path)
        end_time = vehicle.getData().iloc[-1]["timestamp"]

        # Create a trigger condition based on the vehicle's end timestamp
        trigcond = xosc.SimulationTimeCondition(end_time, xosc.Rule.greaterOrEqual)
        trigger = xosc.ValueTrigger(f"Delete{vehicle.getName()}Trigger", 0, xosc.ConditionEdge.rising, trigcond)
        
        # Define the delete action and wrap it inside an event with the trigger
        delete_action = xosc.DeleteEntityAction(vehicle.getName())  # Pass the vehicle name as the entityref
        event_delete = xosc.Event(f"Delete{vehicle.getName()}Event", xosc.Priority.parallel)
        event_delete.add_trigger(trigger)
        event_delete.add_action(f"Delete{vehicle.getName()}Action", delete_action)
        
        # Create a maneuver and a maneuver group to associate the event with the vehicle
        man = xosc.Maneuver(f"Delete{vehicle.getName()}Maneuver")
        man.add_event(event_delete)
        
        mangr = xosc.ManeuverGroup(f"Delete{vehicle.getName()}ManeuverGroup")
        mangr.add_actor(vehicle.getName())
        mangr.add_maneuver(man)
        
        # Define an act with a starting trigger and attach the maneuver group
        starttrigger = xosc.ValueTrigger(f"StartDelete{vehicle.getName()}Condition", 0, xosc.ConditionEdge.rising, xosc.SimulationTimeCondition(0, xosc.Rule.greaterThan))
        act = xosc.Act(f"Delete{vehicle.getName()}Act", starttrigger)
        act.add_maneuver_group(mangr)
        
        return act


    def _createOscVehicleStory(self, vehicle, first_time_value, data_transformation_path_output_dir):
        
        # Reference a trajectory from the catalog using the vehicle's name
        trajectory = xosc.CatalogReference('TrajectoryCatalog', ("PolylineTrajectory_"+vehicle.getName()))

        # Activate trajectory trigger when simulation time >= 0.1s
        trigcond = xosc.SimulationTimeCondition(0.1, xosc.Rule.greaterOrEqual)
        trigger = xosc.ValueTrigger(("ActivateMotionFollowTrajectory"+vehicle.getName()),0,xosc.ConditionEdge.rising, trigcond)
        
        # Define an event that triggers a FollowTrajectory action
        event_traj = xosc.Event(("ActivateEventFollowTrajectory"+vehicle.getName()),xosc.Priority.parallel)
        event_traj.add_trigger(trigger)
        action = xosc.FollowTrajectoryAction(trajectory, following_mode=xosc.FollowingMode.position, reference_domain=xosc.ReferenceContext.absolute, scale=1,  offset=0)
        event_traj.add_action(("ActivateFollowTrajectory"+vehicle.getName()),action)

        # Create a maneuver and maneuver group, and associate them with the vehicle
        man = xosc.Maneuver("Traj" +vehicle.getName()+"MotionManeuver")
        man.add_event(event_traj)

        mangr = xosc.ManeuverGroup(vehicle.getName()+ "MotionManeuverGroup")
        mangr.add_actor(vehicle.getName())
        mangr.add_maneuver(man)

        # Create an act that starts when simulation time > 0s, and add the maneuver group
        starttrigger = xosc.ValueTrigger("Activate"+vehicle.getName()+"AccelerationActCondition",0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
        act = xosc.Act("Activate"+vehicle.getName()+"AccelerationAct",starttrigger)
        act.add_maneuver_group(mangr)

        # Create a story that includes the act and return it
        story = xosc.Story(vehicle.getName() + "FollowTrajectoryStory")
        story.add_act(act)

        return story


    # Create the scenario (main function)
    def create_scenario(self, road_network, output_folder, data_transformation_path_output_dir, username, vehicle_catalog_path):

        # Create catalogs for vehicles and trajectories
        catalog = xosc.Catalog()
        catalog.add_catalog('VehicleCatalog', vehicle_catalog_path)
        catalog.add_catalog('TrajectoryCatalog', output_folder)

        # Define the road network using the provided OpenDRIVE file
        road_network = xosc.RoadNetwork(roadfile=road_network)

        # Define parameters
        paramdec = xosc.ParameterDeclarations()

        # Create scenario entities container
        entities = xosc.Entities()

        # Add each vehicle object to the scenario
        for vehicle in self.__vehicles:
            entities.add_scenario_object(vehicle.getName(), self._createOscVehicle(vehicle))

        # Create Init section of the storyboard for initial vehicle positioning
        init = xosc.Init()

        # Initialize Ego and any other vehicles starting at timestamp 0 
        parquet_file_path = os.path.join(data_transformation_path_output_dir, "Ego_ReSimulationInput.parquet")
        df_ego = pd.read_parquet(parquet_file_path)

        for vehicle in self.__vehicles:
            first_time_value = vehicle.getData().iloc[0]["timestamp"]
            if vehicle.getName() == "Ego" or first_time_value == 0.0:
                trajectory = xosc.CatalogReference('TrajectoryCatalog', "PolylineTrajectory_" + vehicle.getName())
                vehicle_start = xosc.TeleportAction(xosc.TrajectoryPosition(trajectory, first_time_value))
                init.add_init_action(vehicle.getName(), vehicle_start)

        # Determine the simulation stop time based on the latest timestamp across all vehicles
        stop_time = 0.0
        for vehicle in self.__vehicles:
            if vehicle.getData().iloc[-1]["timestamp"] > stop_time:
                stop_time = vehicle.getData().iloc[-1]["timestamp"]       
        
        # Define a stop trigger for the scenario once the maximum vehicle time is reached
        stopcondition = xosc.SimulationTimeCondition(stop_time, xosc.Rule.greaterOrEqual)
        stoptrigger = xosc.ValueTrigger('End', delay=1.0, conditionedge=xosc.ConditionEdge.rising, valuecondition=stopcondition, triggeringpoint="stop")

        # Create storyboard with Init and stop trigger
        storyboard = xosc.StoryBoard(init, stoptrigger)

        for vehicle in self.__vehicles:
            first_time_value = vehicle.getData().iloc[0]["timestamp"]

            # AddEntityAction only for non-Ego vehicles that enter after time 0
            if vehicle.getName() != "Ego" and first_time_value != 0.0:
                storyboard.add_act(self._createAddEntityAction(vehicle, first_time_value))

            # Add trajectory-following story for each vehicle
            storyboard.add_story(self._createOscVehicleStory(vehicle, first_time_value, data_transformation_path_output_dir))

            # Add DeleteEntityAction to clean up the vehicle after its motion ends
            storyboard.add_act(self._createDeleteEntityAction(vehicle, data_transformation_path_output_dir))

        # Construct and save the scenario to the given output folder
        scenario = xosc.Scenario(
            "TrajectoryWithInitialPosition",
            username,
            paramdec,
            entities=entities,
            storyboard=storyboard,
            roadnetwork=road_network,
            catalog=catalog
        )

        scenario_file = os.path.join(output_folder, "drive_scenario.xosc")
        scenario.write_xml(scenario_file)


    def process_file(self, abstraction_path_output_dir, road_network, username, vehicle_catalog_path, output_folder):
        data_dict = {}          # Dictionary to store vehicle data keyed by vehicle name
        vehicles = []           # List to store Vehicle objects
        vehicle_list = os.listdir(abstraction_path_output_dir)
        
        # Parse each file ending with '_ReSimulationInput.parquet'
        for file_name in vehicle_list:
            if file_name.endswith("_ReSimulationInput.parquet"):
                if "Ego_ReSimulationInput" in file_name:
                    data_dict["Ego"] = pd.read_parquet(os.path.join(abstraction_path_output_dir, file_name))
                else:
                    match = re.search(r"([0-9]+[a-zA-Z]*)_ReSimulationInput", file_name)
                    if match:
                        file_id = match.group(1)
                        data_dict[f"Vehicle {file_id}"] = pd.read_parquet(os.path.join(abstraction_path_output_dir, file_name))
                    else:
                        print(f"No match for expected pattern in: {file_name}")

        # Convert dataframes into Vehicle objects and filter out short sequences
        for key, value in data_dict.items():
            data_dict[key] = value

            timestamp_length = len(value["timestamp"])
            if timestamp_length < 10:
                print(f"❌ {key} has only {timestamp_length} timestamps and will be ignored")
                continue

            #create Vehicle object
            vehicles.append(Vehicle(name=key, input_data=value))

        self.setVehicles(vehicles)

        # Generate the OpenSCENARIO file and trajectory catalog
        self.create_scenario(road_network, output_folder, abstraction_path_output_dir, username, vehicle_catalog_path)
        self.create_trajectory_catalog(output_folder, road_network, abstraction_path_output_dir, username)
        