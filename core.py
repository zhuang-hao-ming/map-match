from get_dijkstra_distance import get_dijkstra_distance, MAX_DIS
import networkx as nx

import scipy.spatial as sp
import scipy.stats as stats

SMALL_PROBABILITY = 0.00000001
BIG_PROBABILITY = 0.99999999


def get_transimission_probability(pre_closest_point, closest_point):
    '''
    得到转移概率

    Parameters:
    -----------
    pre_closest_point : CPointRec
        前一个点
    closest_point : CPointRec
        当前点

    Returns:
    ---------
    prob : float
        转移概率
    '''
    # begin_tick = time.time()
    max_distance = (closest_point.log_time - pre_closest_point.log_time).total_seconds() * 33
    max_distance = max_distance if max_distance < MAX_DIS else MAX_DIS
    dijkstra_distance = get_dijkstra_distance(pre_closest_point, closest_point, max_distance)
    # print('get dijkstra distance elapse {}'.format(time.time() - begin_tick))
    euclidean_distance = sp.distance.euclidean([pre_closest_point.log_x, pre_closest_point.log_y], [closest_point.log_x, closest_point.log_y])

    if dijkstra_distance == MAX_DIS:
        return SMALL_PROBABILITY
    if dijkstra_distance > euclidean_distance + 2000:
        return SMALL_PROBABILITY

    if dijkstra_distance == 0:
        return BIG_PROBABILITY

    prob = euclidean_distance / dijkstra_distance
    if prob > BIG_PROBABILITY:
        prob = BIG_PROBABILITY
    if prob < SMALL_PROBABILITY:
        prob = SMALL_PROBABILITY
    
    return prob

def get_observation_probability(closest_point):
    '''
    得到观察概率

    Parameters:
    ----------
    closest_point : CPointRec
        当前点

    '''
    dis = sp.distance.euclidean([closest_point.log_x, closest_point.log_y], [closest_point.p_x, closest_point.p_y])
    return stats.norm.pdf(dis, loc=0, scale=30)

def construct_graph(log_list, log_closest_points):
    '''
    构造权重图

    Parameters:
    ----------
    log_list : list
        组成track的log_id列表
    log_closest_points: dict(list)
        每个log和它对应的closest_points列表

    Returns:
    ---------
    g : nx.Graph
        权重图
    '''
    g = nx.Graph()

    pre_layer = []
    for log_id in log_list:
        closest_points = log_closest_points[log_id]
        assert(len(closest_points) > 0)

        now_layer = []

        for closest_point_idx, closest_point in enumerate(closest_points):
            point_id = str(log_id) + '_' + str(closest_point_idx)
            now_layer.append(point_id)                
            g.add_node(point_id, observation_probability=get_observation_probability(closest_point))
            if len(pre_layer) == 0:
                continue
            else:
                for pre_point_id in pre_layer:
                    pre_log_id, pre_closest_point_idx = pre_point_id.split('_')
                    pre_log_id = int(pre_log_id)
                    pre_closest_point_idx = int(pre_closest_point_idx)
                    pre_closest_point = log_closest_points[pre_log_id][pre_closest_point_idx]
                    transimission_probability = get_transimission_probability(pre_closest_point, closest_point)
                    g.add_edge(pre_point_id, point_id, transimission_probability=transimission_probability)
        pre_layer = now_layer
    return g


def find_match_sequence(g, log_list, log_closest_points):
    '''
    从权重图中， 找到最长路径， 作为结果

    Parameters:
    -----------
    g : nx.Graph
        权重图
    log_ist : list
        组成track的log id
    log_closest_points: dict(list)
        每个log和它对应的closest_points列表

    Returns:
    ----------
    (True, match_list, break_idx)
        是否连通， 组成最长路径的closest point, 如果不连通， 从那个位置开始不连通
    '''
    f = {} # 从开头到当前候选点的的最长路径的长度（最大权重和）
    pre = {} # 记录当前候选点的前一个候选点（最长路径上）
    
    # 记录第一层候选点的权重
    first_log_id = log_list[0]
    for closest_point_idx, closest_point in enumerate(log_closest_points[first_log_id]):
        point_id = str(first_log_id) + '_' + str(closest_point_idx)
        f[point_id] = g.node[point_id]['observation_probability']

    # 记录第二层到最后一层的权重
    pre_log_id = first_log_id
    for now_log_id in log_list[1:]:
        for now_closest_point_idx, now_closest_point in enumerate(log_closest_points[now_log_id]):
            # 遍历当前层的所有候选点
            now_point_id = str(now_log_id) + '_' + str(now_closest_point_idx)
            max_probability = -1
            # 找到从前一次到当前层当前候选点的最大权重
            for pre_closest_point_idx, pre_closest_point in enumerate(log_closest_points[pre_log_id]):
                pre_point_id = str(pre_log_id) + '_' + str(pre_closest_point_idx)
                temp = g[pre_point_id][now_point_id]['transimission_probability'] * g.node[now_point_id]['observation_probability'] + f[pre_point_id]
                if temp > max_probability:
                    max_probability = temp
                    pre[now_point_id] = pre_point_id
            f[now_point_id] = max_probability
        # 更新前一层
        pre_log_id = now_log_id
    
    # 找到权重最大的候选点
    max_probability = -1
    max_point_id = None
    for point_id, probability in f.items():
        if probability > max_probability:
            max_point_id = point_id
            max_probability = probability

    
    assert(max_point_id.split('_')[0] == str(log_list[-1])) # 断言， 概率最大的候选点， 一定在最后一组内
    
    # 从权重最大的候选点， 从尾到头，找到最长路径
    reverse_list = []
    for i in range(1, len(log_list)):
        reverse_list.append(max_point_id)
        max_point_id = pre[max_point_id]
    reverse_list.append(max_point_id)

    # reverse得到路径
    reverse_list.reverse()
    match_list = reverse_list

    # 查看路径中是否存在断点
    break_idx = -1
    for i in range(1, len(match_list)):
        pre_point_id = match_list[i-1]
        now_point_id = match_list[i]
        transimission_probability = g[pre_point_id][now_point_id]['transimission_probability']
        if transimission_probability == SMALL_PROBABILITY:
            break_idx = i
            break

    # 得到每个id对应的候选点信息
    match_point_list = []
    for idx, point_id in enumerate(match_list):
        log_id, closest_point_idx = point_id.split('_')
        log_id = int(log_id)
        assert(log_id == log_list[idx])
        closest_point_idx = int(closest_point_idx)
        match_point_list.append(log_closest_points[log_id][closest_point_idx])

    if break_idx == -1:
        return (True, match_point_list, break_idx)
    else:
        return (False, match_point_list, break_idx)


def match_until_connect(log_list, log_closest_points):
    '''
    尝试构建权重图，获得匹配轨迹， 如果返回的轨迹不连通，
    则删除断裂处的点，重新匹配。

    '''
    cnt = 0
    while True:
        # begin_tick = time.time()
        g = construct_graph(log_list, log_closest_points)
        # print('construct graph for {} logs elapse {}'.format(len(log_list), time.time() - begin_tick))
        # begin_tick = time.time()
        is_connect, match_point_list, break_idx = find_match_sequence(g, log_list, log_closest_points)
        # print('find match for {} logs elapse {}'.format(len(log_list), time.time() - begin_tick))
        if is_connect:
            return match_point_list
        else:
            del log_list[break_idx-1:break_idx+1]
            cnt += 1
        if len(log_list) < 4:
            return None

        if cnt > 10:
            return None