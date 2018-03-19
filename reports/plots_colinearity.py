
import numpy as np
import matplotlib.pyplot as plt

BATCH_PROPS = {}
BATCH_PROPS['20'] = { 'color' : 'g' }
BATCH_PROPS['30'] = { 'color' : 'b' }
BATCH_PROPS['40'] = { 'color' : 'r' }
BATCH_PROPS['50'] = { 'color' : 'c' }


_batchPS3_chessboard = {}
_batchPS3_chessboard['pattern'] = 'chessboard'
_batchPS3_chessboard['results'] = {}
_batchPS3_chessboard['results']['20'] = [ 4.2982, 
                                          4.2996, 4.2996, 4.2996, 4.2994, 4.2995,
                                          4.2996, 4.2995, 4.2995, 4.2996, 4.2995,
                                          4.2997, 4.2996, 4.2996, 4.2995, 4.2996,
                                          4.2995, 4.2996, 4.2996, 4.2995, 4.2995 ]
_batchPS3_chessboard['results']['30'] = [ 4.2628,
                                          4.4368, 4.4368, 4.4368, 4.4368, 4.4368,
                                          4.4279, 4.4278, 4.4277, 4.4277, 4.4278, 
                                          4.4277, 4.4278, 4.4278, 4.4277, 4.4278,
                                          4.4277, 4.4277, 4.4278, 4.4277, 4.4278 ]
_batchPS3_chessboard['results']['40'] = [ 4.1667,
                                          4.1741, 4.1761, 4.1761, 4.1762, 4.1762,
                                          4.1762, 4.1762, 4.1762, 4.1762, 4.1761,
                                          4.1762, 4.1761, 4.1762, 4.1762, 4.1762,
                                          4.1761, 4.1761, 4.1761, 4.1761, 4.1761 ]
_batchPS3_chessboard['results']['50'] = [ 4.9589,
                                          4.9509, 4.9507, 4.9508, 4.9507, 4.9507,
                                          4.9506, 4.9509, 4.9506, 4.9508, 4.9507,
                                          4.9507, 4.9507, 4.9507, 4.9507, 4.9507,
                                          4.9507, 4.9507, 4.9506, 4.9508, 4.9507 ]

_batchPS3_asymmetric = {}
_batchPS3_asymmetric['pattern'] = 'asymmetric'
_batchPS3_asymmetric['results'] = {}
_batchPS3_asymmetric['results']['20'] = [ 3.7070, 
                                          3.7067, 3.7068, 3.7067, 3.7066, 3.7068, 
                                          3.7066, 3.7067, 3.7067, 3.7067, 3.7067,
                                          3.7067, 3.7067, 3.7067, 3.7067, 3.7066,
                                          3.7066, 3.7066, 3.7067, 3.7067, 3.7067 ]
_batchPS3_asymmetric['results']['30'] = [ 3.6823,
                                          3.6820, 3.6820, 3.6820, 3.6820, 3.6820,
                                          3.6820, 3.6820, 3.6820, 3.6820, 3.6820,
                                          3.6820, 3.6820, 3.6820, 3.6820, 3.6820,
                                          3.6820, 3.6820, 3.6821, 3.6820, 3.6820 ]
_batchPS3_asymmetric['results']['40'] = [ 4.3692,
                                          4.3107, 4.3107, 4.3107, 4.3106, 4.3107,
                                          4.3107, 4.3107, 4.3107, 4.3107, 4.3107,
                                          4.3107, 4.3107, 4.3107, 4.3107, 4.3107,
                                          4.3107, 4.3107, 4.3107, 4.3107, 4.3106 ]
_batchPS3_asymmetric['results']['50'] = [ 4.4142,
                                          4.41439, 4.41441, 4.41438, 4.41439, 4.41438,
                                          4.41439, 4.41437, 4.41439, 4.41438, 4.41441,
                                          4.41438, 4.41438, 4.41439, 4.41439, 4.41441,
                                          4.41436, 4.41438, 4.41438, 4.41438, 4.41438 ]

_batchPS3_rings = {}
_batchPS3_rings['pattern'] = 'rings'
_batchPS3_rings['results'] = {}
_batchPS3_rings['results']['20'] = [ 8.1222, 
                                     8.1235, 8.1238, 8.1235, 8.1236, 8.1235, 
                                     8.1235, 8.1236, 8.1237, 8.1235, 8.1236, 
                                     8.1236, 8.1236, 8.1235, 8.1235, 8.1237, 
                                     8.1235, 8.1233, 8.1237, 8.1236, 8.1235 ]
_batchPS3_rings['results']['30'] = [ 8.3407,
                                     8.3420, 8.3420, 8.3420, 8.3422, 8.3420, 
                                     8.3421, 8.3421, 8.3420, 8.3420, 8.3419, 
                                     8.3423, 8.3421, 8.3420, 8.3421, 8.3421, 
                                     8.3419, 8.3417, 8.3419, 8.3420, 8.3419 ]
_batchPS3_rings['results']['40'] = [ 9.8923,
                                     9.8923, 9.8924, 9.8923, 9.8924, 9.8923, 
                                     9.8923, 9.8923, 9.8923, 9.8925, 9.8924, 
                                     9.8924, 9.8924, 9.8925, 9.8925, 9.8922, 
                                     9.8923, 9.8924, 9.8923, 9.8925, 9.8926 ]
_batchPS3_rings['results']['50'] = [ 9.2364,
                                     9.2382, 9.2383, 9.2495, 9.2382, 9.2384, 
                                     9.2495, 9.2493, 9.2381, 9.2383, 9.2383, 
                                     9.2494, 9.2383, 9.2383, 9.2497, 9.2494, 
                                     9.2493, 9.2493, 9.2493, 9.2494, 9.2494 ]


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


plotBatchesResults( _batchPS3_chessboard, 1 )
plotBatchesResults( _batchPS3_asymmetric, 2 )
plotBatchesResults( _batchPS3_rings, 3 )

plt.show()