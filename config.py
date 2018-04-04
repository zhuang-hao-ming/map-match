# 数据库配置
from collections import OrderedDict


crs = {'init': 'epsg:32649'}
driver = 'ESRI Shapefile'

schema = {
    'properties': OrderedDict(
            [                
                ('idx', 'int')                
            ]
        ), 
    'geometry': 'LineString'
    }