
import numpy as np
import matplotlib.pyplot as plt

BATCH_PROPS = {}
BATCH_PROPS['20'] = { 'color' : 'g' }
BATCH_PROPS['30'] = { 'color' : 'b' }
BATCH_PROPS['40'] = { 'color' : 'r' }
BATCH_PROPS['50'] = { 'color' : 'c' }






_batchLifecam_chessboard = {}
_batchLifecam_chessboard['pattern'] = 'chessboard'
_batchLifecam_chessboard['results'] = {}
_batchLifecam_chessboard['results']['20'] = [ 6.4698,
                                              6.4695, 6.46963, 6.46961, 6.46953, 6.46964,
                                              6.46958, 6.46963, 6.46966, 6.46957, 6.46968,
                                              6.46966, 6.46961, 6.4696, 6.46961, 6.46959,
                                              6.46963, 6.46957, 6.46962, 6.46966, 6.46958 ]
_batchLifecam_chessboard['results']['30'] = [ 7.0042,
                                              7.00456, 7.00458, 7.00452, 7.00449, 7.00453,
                                              7.00454, 7.00454, 7.00455, 7.00455, 7.00455,
                                              7.00451, 7.00454, 7.00453, 7.00455, 7.00451,
                                              7.00452, 7.00454, 7.00453, 7.00454, 7.00455 ]
_batchLifecam_chessboard['results']['40'] = [ 4.3315,
                                              4.33912, 4.33896, 4.33887, 4.33892, 4.33888,
                                              4.33889, 4.33888, 4.33903, 4.33886, 4.33885,
                                              4.33876, 4.3389, 4.33889, 4.33889, 4.33884,
                                              4.33912, 4.33878, 4.33914, 4.33888, 4.33893 ]
_batchLifecam_chessboard['results']['50'] = [ 5.4255,
                                              5.37321, 5.37305, 5.37304, 5.37302, 5.37303,
                                              5.37306, 5.37299, 5.37305, 5.37305, 5.37304,
                                              5.37305, 5.37304, 5.37302, 5.37302, 5.37305,
                                              5.37304, 5.37303, 5.37304, 5.37302, 5.37304 ]

_batchLifecam_asymmetric = {}
_batchLifecam_asymmetric['pattern'] = 'asymmetric'
_batchLifecam_asymmetric['results'] = {}
_batchLifecam_asymmetric['results']['20'] = [ 4.0940,
                                              4.09382, 4.0938, 4.09383, 4.09382, 4.09383,
                                              4.09382, 4.09386, 4.09382, 4.09383, 4.09383,
                                              4.0938, 4.09382, 4.09376, 4.09382, 4.09382,
                                              4.09382, 4.09384, 4.09383, 4.09382, 4.09381 ]
_batchLifecam_asymmetric['results']['30'] = [ 5.2767,
                                              5.21826, 5.28927, 5.35867, 5.35864, 5.35857,
                                              5.3586, 5.35863, 5.35861, 5.35859, 5.35862,
                                              5.35858, 5.35862, 5.3586, 5.35862, 5.35863,
                                              5.3586, 5.35861, 5.3586, 5.35859, 5.3586 ]
_batchLifecam_asymmetric['results']['40'] = [ 4.6660,
                                              4.66603, 4.66603, 4.66601, 4.66602, 4.66602,
                                              4.66602, 4.666, 4.66601, 4.66602, 4.666,
                                              4.666, 4.66603, 4.666, 4.66603, 4.66601,
                                              4.66603, 4.66601, 4.66601, 4.666, 4.66603 ]
_batchLifecam_asymmetric['results']['50'] = [ 4.3502,
                                              4.28751, 4.28742, 4.28741, 4.28741, 4.28745,
                                              4.28743, 4.28743, 4.28742, 4.28743, 4.28743,
                                              4.28742, 4.28741, 4.28743, 4.28744, 4.28745,
                                              4.28742, 4.28742, 4.28742, 4.28741, 4.28743 ]


_batchLifecam_rings = {}
_batchLifecam_rings['pattern'] = 'rings'
_batchLifecam_rings['results'] = {}
_batchLifecam_rings['results']['20'] = [ 7.23147,
                                         7.33087, 7.36335, 7.36344, 7.33112, 7.36303,
                                         7.3305, 7.36357, 7.33015, 7.36327, 7.33089,
                                         7.36299, 7.36308, 7.36374, 7.33124, 7.36346,
                                         7.33062, 7.36347, 7.36299, 7.36357, 7.36297 ]
_batchLifecam_rings['results']['30'] = [ 7.1577,
                                         7.31288, 7.31292, 7.31279, 7.31276, 7.31311,
                                         7.31328, 7.31324, 7.31316, 7.31285, 7.31304,
                                         7.31323, 7.31292, 7.3129, 7.31289, 7.31312,
                                         7.31299, 7.313, 7.31286, 7.31303, 7.31293 ]
_batchLifecam_rings['results']['40'] = [ 7.6354,
                                         7.75455, 7.75429, 7.75481, 7.81075, 7.81077,
                                         7.75442, 7.75443, 7.75449, 7.7546, 7.75462,
                                         7.81061, 7.7546, 7.75437, 7.75442, 7.81093,
                                         7.75447, 7.75491, 7.81083, 7.81083, 7.81084 ]
_batchLifecam_rings['results']['50'] = [ 8.0049,
                                         8.21077, 8.21079, 8.21082, 8.21094, 8.21066,
                                         8.21091, 8.21091, 8.21087, 8.21081, 8.21086,
                                         8.21085, 8.21083, 8.21087, 8.2109, 8.21099,
                                         8.21077, 8.21078, 8.21098, 8.21084, 8.21095 ]


def plotBatchesResults( batch, figureIndx ) :

    plt.figure( figureIndx )

    for key in BATCH_PROPS :

        _color = BATCH_PROPS[key]['color']
        _results = batch['results'][key]
        _pattern = batch['pattern']

        _label = _pattern + " - " + key

        ii = [ ( i + 1 ) for i in range( len( _results ) ) ]
        yy = [ _res for _res in _results ]

        plt.plot( ii, yy, _color, label = _label )

    plt.legend( loc = 'lower right', shadow = True )
    plt.grid( True )


plotBatchesResults( _batchLifecam_chessboard, 1 )
plotBatchesResults( _batchLifecam_asymmetric, 2 )
plotBatchesResults( _batchLifecam_rings, 3 )

plt.show()
