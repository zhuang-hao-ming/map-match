# -*- coding: utf-8 -*-
'''

地图匹配



'''
import time
from datetime import datetime
from collections import namedtuple, defaultdict, OrderedDict


import psycopg2
import fiona
from shapely.geometry import shape, Point
from shapely.strtree import STRtree

from config import crs, driver, schema

from core import match_until_connect
from get_dijkstra_distance import get_connected_path
from cache import clear_cache

CPointRec = namedtuple('CPointRec', ["log_x", "log_y", "p_x", "p_y", "road_id", "log_id", "source", "target", "weight", "fraction", "v", "log_time", "track_id", "car_id"])
TrackRec = namedtuple('TrackRec', ['x','y', 'uuid', 'track_id', 'log_time', 'car_id', 'v'])



def get_road_rtree(shp_path):
    '''
    获得道路rtree，coord->feature字典

    Parameters:
    -----------
    shp_path : str
        道路文件名

    '''
    c = fiona.open(shp_path)

    coord_feature_dict = {}
    geom_list = []
    
    for feature in c:
        geometry = feature['geometry']
        geom = shape(geometry)
        geom_list.append(geom)

        coord_key = geom.coords[0] + geom.coords[-1]
        assert(coord_key not in coord_feature_dict)
        coord_feature_dict[coord_key] = feature

    c.close()

    rtree = STRtree(geom_list)

    return rtree, coord_feature_dict
    


def get_closest_points(log, road_rtree, coord_feature_dict):
    '''
    获得点在路网中的投影点

    Parameters:
    -------------
    point : shapely point
        gps log点
    road_tree : shapely rtree
        道路rtree
    coord_feature_dict : dict
        道路头尾坐标 -> 道路feature字典

    '''
    # begin_tick = time.time()
    point = Point(log.x, log.y)
    point_buffer = point.buffer(30)
    project_roads = []
    for road in road_rtree.query(point_buffer):
        if road.intersects(point_buffer):
            project_roads.append(road)

    project_points = []
    for road in project_roads:
        fraction = road.project(point, normalized=True)
        project_point = road.interpolate(fraction, normalized=True)
        road_feature = coord_feature_dict[road.coords[0]+road.coords[-1]]        
        project_points.append(CPointRec(
            log.x,
            log.y,
            project_point.x,
            project_point.y,
            int(road_feature['id']),
            log.uuid,
            road_feature['properties']['source'],
            road_feature['properties']['target'],
            road_feature['properties']['weight'],
            fraction,
            log.v,
            log.log_time,
            log.track_id,
            log.car_id
        ))
    
    # print('get_closest_points time: {}'.format(time.time() - begin_tick))
    
    return project_points

def read_track(shp_path):
    '''
    '''
    track_id_logs = defaultdict(list)

    c = fiona.open(shp_path)
    for feature in c:
        geometry = feature['geometry']
        x = geometry['coordinates'][0]
        y = geometry['coordinates'][1]
        properties = feature['properties']
        track_id = properties['track_id']
        track_id_logs[track_id].append(
            TrackRec(
                x,
                y,
                properties['uuid'],
                track_id,
                datetime.strptime(properties['log_time'], '%Y-%m-%d %H:%M:%S'),
                properties['car_id'],
                properties['v']
                )
            )
    
    return track_id_logs


def read_road(shp_path='./shp/input/connected_road.shp'):
    '''
    读road文件，
    获得，(source, target) -> road_id 字典
    和 road_id -> geometry字典

    
    '''
    c = fiona.open(shp_path)

    road_id_geometry_dict = {}
    key_road_id_dict = {}

    for feature in c:
        properties = feature['properties']
        source = properties['source']
        target = properties['target']
        road_id = int(feature['id'])

        road_id_geometry_dict[road_id] = feature['geometry']
        key_road_id_dict[((source, target))] = road_id

    return key_road_id_dict, road_id_geometry_dict

if __name__ == '__main__':

    road_rtree, coord_feature_dict = get_road_rtree('./shp/input/connected_road.shp')
    
    key_road_id_dict, road_id_geometry_dict = read_road('./shp/input/connected_road.shp')


    # track_id -> logs 字典
    track_id_logs = read_track('./shp/input/track.shp')    
    for track_id, logs in track_id_logs.items():
        begin_tick = time.time()        
        log_id_list = [log.uuid for log in logs]
        log_closest_points = defaultdict(list)
                
        for log in logs:
            project_points = get_closest_points(log, road_rtree, coord_feature_dict)            
            log_closest_points[log.uuid] = project_points

        clear_cache()
        match_point_list = match_until_connect(log_id_list, log_closest_points)
        if match_point_list is not None:
            connected_vertex_path, connected_road_path = get_connected_path(match_point_list)
            if connected_vertex_path is not None:
                assert(connected_road_path is not None)                
                
                
                out_c = fiona.open('./shp/output/new_path{}.shp'.format(track_id), 'w', driver=driver, crs=crs, schema=schema)                
                for i, road_id in enumerate(connected_road_path):
                    rec = {
                        'type': 'Feature',
                        'id': '-1',
                        'geometry': road_id_geometry_dict[int(road_id)],
                        'properties': OrderedDict([
                            ('idx', i)
                        ])
                    }
                    out_c.write(rec)
                out_c.close()


                # out_c = fiona.open('./shp/output/path{}.shp'.format(track_id), 'w', driver=driver, crs=crs, schema=schema)                
                
                # for i in range(2, len(connected_vertex_path)):
                #     pre_point = connected_vertex_path[i-1]
                #     now_point = connected_vertex_path[i]
                #     assert((pre_point, now_point) in key_road_id_dict)
                #     rec = {
                #         'type': 'Feature',
                #         'id': '-1',
                #         'geometry': road_id_geometry_dict[key_road_id_dict[(pre_point, now_point)]],
                #         'properties': OrderedDict([
                #             ('idx', i)
                #         ])
                #     }
                #     out_c.write(rec)
                # out_c.close()

        print(time.time()-begin_tick)
        
        
        

    


    # 保存结果