#! /usr/bin/env python3
import roslib, rospy, rosnode, rosparam, os, time, subprocess, yaml
from webclient.srv import *
import rospkg
import psycopg2
import json
import zlib
import threading

# Database connection parameters
DB_NAME = rospy.get_param('/agv/database/DB_NAME')
DB_USER = rospy.get_param('/agv/database/DB_USER')
DB_PASSWORD = rospy.get_param('/agv/database/DB_PASSWORD')
DB_HOST = rospy.get_param('/agv/database/DB_HOST')
DB_PORT = rospy.get_param('/agv/database/DB_PORT')
################################
# robot name
AGV_NAME = rospy.get_param('/agv/robot_id')
################################

## Path file ##
rospack = rospkg.RosPack()
package_path = rospack.get_path('webclient')
mappath = package_path + '/Map/'
configpath = package_path + '/robot_config/'
defaultMap = configpath + 'defaultMap.yaml'
###########################

## Tools ##
def kill_nodes(node_list):
    running_nodes = rosnode.get_node_names()
    nodes_to_kill = [node for node in running_nodes if any(node.startswith(name) for name in node_list)]

    # Create a copy of the list to iterate over
    nodes_to_kill_copy = list(nodes_to_kill)

    for node in nodes_to_kill_copy:
        if node not in running_nodes:
            nodes_to_kill.remove(node)

    success, fail = rosnode.kill_nodes(nodes_to_kill)

    if success:
        rospy.logwarn('Successfully killed nodes: %s', success)
    if fail:
        rospy.logwarn('Failed to kill nodes on the first attempt: %s', fail)
    
    rospy.sleep(1.0)

    i = 0
    while i < 10 and len(fail) > 0:
        running_nodes = rosnode.get_node_names()

        # Create a copy of the list to iterate over
        fail_copy = list(fail)

        for node in fail_copy:
            if node not in running_nodes:
                fail.remove(node)

        success, fail = rosnode.kill_nodes(fail)

        if success:
            rospy.logwarn('Successfully killed nodes: %s', success)
        if fail:
            rospy.logwarn('Failed to kill nodes: %s', fail)
        
        rospy.sleep(0.2)
        i += 1

    if len(fail) > 0:
        rospy.logwarn('Failed to kill nodes after multiple attempts: %s', fail)
        rospy.sleep(2.0)

    return fail

###########################

# Check Map Consistency #
def check_map_consistency_periodic():
    while not rospy.is_shutdown():
        try:
            response = check_map_consistency(MapConsistencyCheckRequest())
            if response.are_consistent:
                rospy.loginfo("Local and database maps are consistent.")
            else:
                rospy.logwarn("Local and database maps are inconsistent.")
                rospy.logwarn("Missing in DB: %s", response.missing_in_db)
                rospy.logwarn("Missing in Local: %s", response.missing_in_local)

                # Automatically download missing maps from the database
                for map_name in response.missing_in_local:
                    try:
                        download_response = download_map_handler(DownloadMapRequest(map_name=map_name))
                        if download_response.result == "Success":
                            rospy.loginfo("Map '%s' downloaded successfully from the database.", map_name)
                        else:
                            rospy.logerr("Error downloading map '%s': %s", map_name, download_response.result)
                    except Exception as e:
                        rospy.logerr("Error during map download: %s", str(e))
          
        except Exception as e:
            rospy.logerr("Error during map consistency check: %s", str(e))
        
        rospy.sleep(5.0)

def check_map_consistency(req):
    try:
        # Connect to the database
        connection = psycopg2.connect(dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST, port=DB_PORT)
        cursor = connection.cursor()

        # Retrieve the list of map names from the database
        cursor.execute("SELECT map_name FROM maps;")
        # Get the map names from the database
        db_map_names = [result[0] for result in cursor.fetchall()]

        # Close the cursor and connection
        cursor.close()
        connection.close()

        # Get map names from the local directory
        local_map_names = [x[:-5] for x in os.listdir(mappath) if x.endswith('.yaml')]

        # Check if the local map and the map in the database are the same
        missing_in_db = [name for name in local_map_names if name not in db_map_names]
        missing_in_local = [name for name in db_map_names if name not in local_map_names]

        # Prepare the response
        response = MapConsistencyCheckResponse()
        response.are_consistent = not (missing_in_db or missing_in_local)
        response.error_message = "Maps are consistent."
        response.missing_in_db = missing_in_db
        response.missing_in_local = missing_in_local

        return response

    except Exception as e:
        rospy.logerr('Error during map consistency check: %s', str(e))
        response = MapConsistencyCheckResponse()
        response.are_consistent = False
        response.error_message = 'Error: ' + str(e)
        response.missing_in_db = []
        response.missing_in_local = []
        return response

###########################
    
## Get Map List ##
def get_map_list_handler(req):
    try:
        connection = psycopg2.connect(dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST, port=DB_PORT)
        cursor = connection.cursor()

        # Retrieve the list of map names
        cursor.execute("SELECT map_name FROM maps;")
        map_names = [result[0] for result in cursor.fetchall()]

        cursor.close()
        connection.close()

        return MapListingResponse(map_names)

    except Exception as e:
        rospy.logerr('Error while getting map list: %s', str(e))
        return MapListingResponse([e.message])

def map_listing(req):
    try:
        maplisting = [ x for x in os.listdir(mappath) if x.endswith('.yaml')]
        maplisting.sort()
        return MapListingResponse(maplisting)
    except Exception as e:
        rospy.logerr('map_listing exception: %s', str(e))
        return MapListingResponse([e.message])
    
###############################
    
## Upload, Download and remove Map ##
def upload_map_handler(req):
    try:
        map_name = req.map_name
        map_file_path = f'{mappath}/{map_name}.yaml'
        pgm_file_path = f'{mappath}/{map_name}.pgm'

        # Load map data from YAML file
        with open(map_file_path, 'r') as file:
            map_data = yaml.load(file, Loader=yaml.FullLoader)

        # Read the content of the .pgm file
        with open(pgm_file_path, 'rb') as pgm_file:
            pgm_content = pgm_file.read()

        # Create a dictionary with extracted values
        info_dict = {
            'free_thresh': map_data.get('free_thresh', 0.0),
            'image': pgm_file_path,  
            'occupied_thresh': map_data.get('occupied_thresh', 0.0),
            'origin': map_data.get('origin', [0.0, 0.0, 0.0]),
            'negate': map_data.get('negate' ,0.0),
            'resolution': map_data.get('resolution', 0.0),
        }

        # Convert info_dict to JSON
        info_json = json.dumps(info_dict)

        # Upload the map data to PostgreSQL
        connection = psycopg2.connect(dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST, port=DB_PORT)
        cursor = connection.cursor()

        # Check if the map already exists
        cursor.execute("SELECT EXISTS(SELECT 1 FROM maps WHERE map_name = %s LIMIT 1);", (map_name,))
        map_exists = cursor.fetchone()[0]


        if map_exists:
            # Update the existing map entry
            update_query = """
                UPDATE maps
                SET map_data = %s, map_metadata = %s
                WHERE map_name = %s
            """
            cursor.execute(update_query, (psycopg2.Binary(pgm_content), info_json, map_name))
        else:
            # Insert a new map entry
            insert_query = """
                INSERT INTO maps (map_name, map_data, map_metadata)
                VALUES (%s, %s, %s)
            """
            cursor.execute(insert_query, (map_name, psycopg2.Binary(pgm_content), info_json))

        connection.commit()
        cursor.close()
        connection.close()

        return UploadMapResponse(result='Success')

    except Exception as e:
        return UploadMapResponse(result='Error: ' + str(e))

def download_map_handler(req):
    try:
        map_name = req.map_name

        connection = psycopg2.connect(dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST, port=DB_PORT)
        cursor = connection.cursor()

        cursor.execute("SELECT map_data, map_metadata FROM maps WHERE map_name = %s;", (map_name,))
        result = cursor.fetchone()

        if not result:
            raise FileNotFoundError("Map with name '%s' not found in the database." % map_name)

        map_data, map_metadata = result

        try:
            decompressed_data = zlib.decompress(map_data)
        except zlib.error:
            decompressed_data = map_data

        pgm_content = decompressed_data

        if isinstance(map_metadata, dict):
            map_metadata = json.dumps(map_metadata)

        map_metadata_dict = yaml.safe_load(map_metadata)
        # Convert float values to integers
        yaml.SafeDumper.org_represent_str = yaml.SafeDumper.represent_str
        yaml.SafeDumper.represent_str = lambda self, data: self.org_represent_str(str(data))

        yaml_metadata = yaml.dump(map_metadata_dict, default_flow_style=False)

        pgm_full_path = os.path.join(mappath, map_name + '.pgm')
        yaml_full_path = os.path.join(mappath, map_name + '.yaml')

        with open(pgm_full_path, "wb") as pgm_file:
            pgm_file.write(pgm_content)

        with open(yaml_full_path, "w") as yaml_file:
            yaml_file.write(yaml_metadata)

        response = DownloadMapResponse()
        response.result = "Success"

        cursor.close()
        connection.close()

        return response

    except Exception as e:
        rospy.logerr('Error during map download: %s' % str(e))
        return DownloadMapResponse(result='Error: %s' % str(e))

def remove_map_local_handler(req):
    try:
        map_name = req.map_name

        local_map_path_yaml = os.path.join(mappath, "%s.yaml", map_name)
        local_map_path_pgm = os.path.join(mappath, "%s.pgm", map_name)

        if os.path.exists(local_map_path_yaml) or os.path.exists(local_map_path_pgm):
            os.remove(local_map_path_yaml)
            os.remove(local_map_path_pgm)
            rospy.loginfo("Map '%s' removed from the local directory.", map_name)
            return RemoveMapResponse_localResponse("Map '%s' removed from the local directory.", map_name)
        else:
            rospy.logwarn("Map '%s' not found in the local directory.", map_name)
            return RemoveMapResponse_localResponse("Map '%s' not found in the local directory.", map_name)
    except Exception as e:
        rospy.logerr("Error removing map '%s' from the local directory: %s", map_name, str(e))
        return RemoveMapResponse_localResponse("Error removing map '%s' from the local directory: %s" % (map_name, str(e)))

def remove_map_db_handler(req):
    try:
        map_name = req.map_name

        # Use a context manager (with statement) to handle connection and cursor
        with psycopg2.connect(dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST, port=DB_PORT) as connection:
            with connection.cursor() as cursor:
                cursor.execute("DELETE FROM maps WHERE map_name = %s;", (map_name,))
                connection.commit()

        rospy.loginfo(f"Map '{map_name}' removed from the database successfully.")
        return RemoveMapResponse_dbResponse(f"Map '{map_name}' removed from the database successfully.")

    except Exception as e:
        rospy.logerr("Error during map removal from database: %s", str(e))
        return RemoveMapResponse_dbResponse("Error during map removal from database: %s" % str(e))
###############################

## Save and Set Map ##
def map_saver(req):
    global mappath
    global configpath
    rospy.loginfo('Start saving map...')
    os.chdir(mappath)
    mapfilename = req.file_name

    ## Save map to file to lcoal directory ##
    os.system('rosrun map_server map_saver -f %s' % mapfilename)
    rospy.sleep(2)
    
    ## Save map to db ##
    upload_map_handler(UploadMapRequest(map_name=mapfilename))

    ## set map to default ##
    rospy.set_param('/agv/mapfile', '%s/%s.yaml' % (mappath, mapfilename))
    rosparam.dump_params('%s/defaultMap.yaml' % configpath, '/agv/mapfile')
    return SaveMapResponse('Done')

def set_map(req):
    os.chdir(mappath)
    map_cfg = req.map_name
    # Check if map file exists locally
    map_cfg_full = os.path.join(mappath, map_cfg)
    map_cfg_full_yaml = map_cfg_full + '.yaml'
    map_cfg_full_pgm = map_cfg_full + '.pgm'
    rospy.logerr(map_cfg_full)
    if not os.path.exists(map_cfg_full_yaml and map_cfg_full_pgm):
        rospy.logwarn("Map '%s' not found locally. Checking database..." % map_cfg)
        # Download map from database if not found locally
        try:
            download_response = download_map_handler(DownloadMapRequest(map_name=map_cfg))
            if download_response.result == "Success":
                rospy.loginfo(f"Map '{map_cfg}' downloaded successfully from the database.")
                map_cfg_full = os.path.join(mappath, map_cfg)  # Refresh path after download
            else:
                rospy.logerr(f"Error downloading map '{map_cfg}': {download_response.result}")
                raise SetMapResponse(f"Map '{map_cfg}' not available.")
        except Exception as e:
            rospy.logerr(f"Error during map download: {str(e)}")
            raise SetMapResponse(f"Map '{map_cfg}' not available.")

    # Set map as usual if found locally or downloaded successfully
    rospy.set_param('/agv/mapfile', map_cfg_full_yaml)
    rosparam.dump_params('%s/defaultMap.yaml' % configpath, '/agv/mapfile')
    return SetMapResponse('Succeeded set Map')

###############################

## Check Database Connection ##
def check_database_connection():
    try:
        # Attempt to establish a connection to the database
        connection = psycopg2.connect(dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST, port=DB_PORT)
        connection.close()
        rospy.loginfo('Connected to the database successfully.')
        return True
    except Exception as e:
        rospy.logerr('Error connecting to the database: %s', str(e))
        return False

###############################

def map_saver_server():
    rospy.init_node('agv_gmap_saver')
    
    if check_database_connection():
        s1 = rospy.Service('agv_map_saver', SaveMap, map_saver) #map_saver
        s2 = rospy.Service('agv_map_list_local', MapListing, map_listing) #map_listing
        s3 = rospy.Service('agv_set_map', SetMap, set_map) #set_map
        s4 = rospy.Service('agv_upload_map_db', UploadMap, upload_map_handler)  #upload_map
        s5 = rospy.Service('agv_map_list_db', MapListing, get_map_list_handler) #get_map_list
        s6 = rospy.Service("agv_download_map_db", DownloadMap, download_map_handler) #download_map
        s7 = rospy.Service('agv_check_map_consistency', MapConsistencyCheck, check_map_consistency) #check_map_consistency
        s8 = rospy.Service('agv_remove_map_local', RemoveMapResponse_local, remove_map_local_handler) #remove_map_local
        s9 = rospy.Service('agv_remove_map_db', RemoveMapResponse_db, remove_map_db_handler) #remove_map_db
        # # Start a thread for periodic map consistency check
        consistency_thread = threading.Thread(target=check_map_consistency_periodic)
        consistency_thread.start()

        rospy.loginfo('map admin server is ready ...')
        rospy.spin()
    else:
        rospy.logerr('Unable to connect to the database. Exiting...')
        rospy.signal_shutdown('Unable to connect to the database.')


if __name__ == '__main__':
    try:
        map_saver_server()
    except Exception as e:
        rospy.logerr('map_saver_server exception: %s', str(e))

