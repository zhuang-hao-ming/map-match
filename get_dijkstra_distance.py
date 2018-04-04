
'''

获得两点间的dijkstra距离

'''

print('load road')

from collections import namedtuple

import networkx as nx
import psycopg2
import fiona


from cache import get_distance_from_cache, save_distance_to_cache, get_unique_id


CPointRec = namedtuple('CPointRec', ["log_x", "log_y", "p_x", "p_y", "road_id", "log_id", "source", "target", "weight", "fraction", "v", "log_time", "track_id", "car_id"])






MAX_V = 33
MAX_DIS = 5000
ROAD_GRAPH = None




def init_road_graph():
    global ROAD_GRAPH
    c = fiona.open('./shp/input/connected_road.shp')
    road_graph = nx.DiGraph()    
    for feature in c:
        road_id = int(feature['id'])
        properties = feature['properties']
        source = int(properties['source'])
        target = int(properties['target'])
        weight = float(properties['weight'])
        road_graph.add_edge(source, target, weight=weight, road_id=road_id)
    
    ROAD_GRAPH = road_graph




def get_dijkstra_distance(pre_closest_point, now_closest_point, cufoff=5000):
    
    if (ROAD_GRAPH is None):
        print('init road_graph')
        init_road_graph()


    '''
    获得两个点之间的dijkstra距离

    如果两个点之间的距离>cufoff，则认为两点之间的距离为MAX_DIS，这个操作是为了提高效率。

    Parameters:
    -----------
    pre_closest_point : CPointRec
        起点
    now_closest_point : CPointRec
        终点

    '''
    pre_road_id = pre_closest_point.road_id
    pre_source = pre_closest_point.source
    pre_target = pre_closest_point.target
    pre_fraction = pre_closest_point.fraction
    pre_weight = pre_closest_point.weight
    
    assert(ROAD_GRAPH[pre_source][pre_target]['weight'] == pre_weight)


    now_road_id = now_closest_point.road_id
    now_source = now_closest_point.source
    now_target = now_closest_point.target
    now_fraction = now_closest_point.fraction
    now_weight = now_closest_point.weight

    assert(ROAD_GRAPH[now_source][now_target]['weight'] == now_weight)
    
    source_id = get_unique_id(pre_road_id, pre_fraction) # 唯一标识一个起点
    target_id = get_unique_id(now_road_id, now_fraction) # 唯一标识一个终点
    
    

    # if cached
    result = get_distance_from_cache(source_id, target_id)
    if result:
        # print('from cache')
        return result[0]

    # if not cached
    if pre_road_id == now_road_id:
        if now_fraction <= pre_fraction:
            save_distance_to_cache(source_id, target_id, MAX_DIS, None, None)
            return MAX_DIS
        else:
            dis = (now_fraction-pre_fraction) * now_weight
            save_distance_to_cache(source_id, target_id, dis, ['a', 'b'], [pre_road_id])
            return dis


    


    pre_id = 'a'
    now_id = 'b'

    if pre_fraction == 0:
        pre_id = pre_source
    elif pre_fraction == 1:
        pre_id = pre_target
    else:
        ROAD_GRAPH.add_edge(pre_source, pre_id, weight = pre_fraction * pre_weight,road_id=pre_road_id)
        ROAD_GRAPH.add_edge(pre_id, pre_target, weight = (1-pre_fraction) * pre_weight,road_id=pre_road_id)

    if now_fraction == 0:
        now_id = now_source
    elif now_fraction == 1:
        now_id = now_target
    else:
        ROAD_GRAPH.add_edge(now_source, now_id, weight = now_fraction * now_weight, road_id=now_road_id)
        ROAD_GRAPH.add_edge(now_id, now_target, weight = (1-now_fraction) * now_weight, road_id=now_road_id)
    
    dis = MAX_DIS
    vertex_path = None

    length, path = nx.single_source_dijkstra(ROAD_GRAPH, pre_id, cutoff=cufoff)
    try:
        dis = length[now_id]
        vertex_path = path[now_id]
    except KeyError:
        pass

    if vertex_path is None:
        save_distance_to_cache(source_id, target_id, dis, None, None)
    else:
        road_path = ['x']
        for i in range(1, len(vertex_path)):
            pre_vertex = vertex_path[i-1]
            now_vertex = vertex_path[i]
            road_id = ROAD_GRAPH[pre_vertex][now_vertex]['road_id']
            if road_id != road_path[-1]:
                road_path.append(road_id)

        save_distance_to_cache(source_id, target_id, dis, vertex_path, road_path[1:])


    if pre_fraction != 0 and pre_fraction != 1:
        ROAD_GRAPH.remove_edge(pre_source, pre_id)
        ROAD_GRAPH.remove_edge(pre_id, pre_target)

    if now_fraction != 0 and now_fraction != 1:
        ROAD_GRAPH.remove_edge(now_source, now_id)
        ROAD_GRAPH.remove_edge(now_id, now_target)
    
    return dis


def get_connected_path(match_point_list):
    '''
    获得match_list对应的connected vertex path和connected road path

    Parameters:
    -------------
    match_point_list : list
        匹配好的点列表
    
    Returns:
    ----------
    connected_vertex_path : list
        轨迹按顺序经过的vertex
    connected_road_path : list
        轨迹按顺序经过的road

    '''

    pre_point = match_point_list[0]
    connected_vertex_path = ['x']
    connected_road_path = ['x']

    for now_point in match_point_list[1:]:
        
        source_id = get_unique_id(pre_point.road_id, pre_point.fraction)
        target_id = get_unique_id(now_point.road_id, now_point.fraction)
        
        result = get_distance_from_cache(source_id, target_id)        
        assert(result is not None)

        dis, vertex_path, road_path = result

        assert(vertex_path is not None)
        assert(road_path is not None)

        elapse_time = (now_point.log_time - pre_point.log_time).total_seconds()
        if elapse_time * MAX_V < dis:
            return None, None # 超速行驶

        for vertex in vertex_path:
            if vertex in ['a', 'b']:
                continue
            else:
                if vertex != connected_vertex_path[-1]:
                    connected_vertex_path.append(int(vertex))
        for road in road_path:
            if road != connected_road_path[-1]:
                connected_road_path.append(int(road))
        pre_point = now_point

    return connected_vertex_path[1:], connected_road_path[1:]













if __name__ == '__main__':
    # "log_x", "log_y", "p_x", "p_y", "road_id", "log_id", "source", "target", "weight", "fraction", "v", "log_time", "track_id"
    # a = CPointRec(-1, -1, -1, -1, road_id, -1, source, target, weight, fraction, -1, -1, -1)

    a = CPointRec(-1, -1, -1, -1, 5933, -1, 2412, 2413, 155.541266283945987, 0.5, -1, -1, -1, -1)
    b = CPointRec(-1, -1, -1, -1, 5933, -1, 2412, 2413, 155.541266283945987, 0.6, -1, -1, -1, -1)
    assert(get_dijkstra_distance(a, b) == 15.554126628394595)
    
    result = get_distance_from_cache(get_unique_id(5933, 0.5), get_unique_id(5933, 0.6))
    assert(result[0] == 15.554126628394595 and result[1] == ['a', 'b'] and result[2] == [5933])

    assert(get_dijkstra_distance(a, b) == 15.554126628394595) # from cache


    a = CPointRec(-1, -1, -1, -1, 5933, -1, 2412, 2413, 155.541266283945987, 0, -1, -1, -1, -1)
    b = CPointRec(-1, -1, -1, -1, 5933, -1, 2412, 2413, 155.541266283945987, 0.6, -1, -1, -1, -1)
    assert(get_dijkstra_distance(a, b) == 155.541266283945987 * 0.6)

    a = CPointRec(-1, -1, -1, -1, 5933, -1, 2412, 2413, 155.541266283945987, 1, -1, -1, -1, -1)
    b = CPointRec(-1, -1, -1, -1, 5933, -1, 2412, 2413, 155.541266283945987, 0.5, -1, -1, -1, -1)
    assert(get_dijkstra_distance(a, b) == MAX_DIS)

    a = CPointRec(-1, -1, -1, -1, 31222, -1, 32697, 32714, 343.795168360553987, 0.5, -1, -1, -1, -1)
    b = CPointRec(-1, -1, -1, -1, 63796, -1, 32714, 40182, 144.726173089272010, 0.5, -1, -1, -1, -1)
    assert(get_dijkstra_distance(a,b) == 244.26067072491298)

    result = get_distance_from_cache(get_unique_id(31222, 0.5), get_unique_id(63796, 0.5))
    
    assert(result[0] == 244.26067072491298 and result[1] == ['a', 32714, 'b'] and result[2] == [31222, 63796])

    assert(get_dijkstra_distance(a, b) == 244.26067072491298) # from cache

    a = CPointRec(-1, -1, -1, -1, 31222, -1, 32697, 32714, 343.795168360553987, 0, -1, -1, -1, -1)
    b = CPointRec(-1, -1, -1, -1, 63796, -1, 32714, 40182, 144.726173089272010, 1, -1, -1, -1, -1)
    assert(get_dijkstra_distance(a,b) == 343.795168360553987+144.726173089272010)

    result = get_distance_from_cache(get_unique_id(31222, 0), get_unique_id(63796, 1))    
    assert(result[0] == 343.795168360553987+144.726173089272010 and result[1] == [32697, 32714, 40182] and result[2] == [31222, 63796])
    
    a = CPointRec(-1, -1, -1, -1, 31222, -1, 32697, 32714, 343.795168360553987, 0, -1, -1, -1, -1)
    b = CPointRec(-1, -1, -1, -1, 63796, -1, 32714, 40182, 144.726173089272010, 0, -1, -1, -1, -1)
    assert(get_dijkstra_distance(a,b) == 343.795168360553987)

    result = get_distance_from_cache(get_unique_id(31222, 0), get_unique_id(63796, 0))    
    assert(result[0] == 343.795168360553987 and result[1] == [32697, 32714])

    a = CPointRec(-1, -1, -1, -1, 31222, -1, 32697, 32714, 343.795168360553987, 1, -1, -1, -1, -1)
    b = CPointRec(-1, -1, -1, -1, 63796, -1, 32714, 40182, 144.726173089272010, 0, -1, -1, -1, -1)
    assert(get_dijkstra_distance(a,b) == 0)
    result = get_distance_from_cache(get_unique_id(31222, 1), get_unique_id(63796, 0))    
    assert(result[0] == 0 and result[1] == [32714])
    assert(get_dijkstra_distance(a,b) == 0) ## from cache
    

    a = CPointRec(-1, -1, -1, -1, 31222, -1, 32697, 32714, 343.795168360553987, 1, -1, -1, -1, -1)
    b = CPointRec(-1, -1, -1, -1, 63796, -1, 32714, 40182, 144.726173089272010, 1, -1, -1, -1, -1)
    assert(get_dijkstra_distance(a,b) == 144.726173089272010)


    

    

    # 测试！