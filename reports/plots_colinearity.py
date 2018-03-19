
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

# _batchPS3_asymmetric = {}
# _batchPS3_asymmetric['pattern'] = 'asymmetric'
# _batchPS3_asymmetric['results'] = {}
# _batchPS3_asymmetric['results']['20'] = [ 0.1914, 
#                                           0.1937, 0.1940, 0.1936, 0.1939, 0.1934,
#                                           0.1931, 0.1929, 0.1932, 0.1932, 0.1933,
#                                           0.1932, 0.1933, 0.1934, 0.1944, 0.1938,
#                                           0.1933, 0.1934, 0.1930, 0.1936, 0.1933 ]
# _batchPS3_asymmetric['results']['30'] = [ 0.1848,
#                                           0.1864, 0.1863, 0.1863, 0.1862, 0.1864,
#                                           0.1863, 0.1862, 0.1862, 0.1864, 0.1860,
#                                           0.1863, 0.1863, 0.1864, 0.1866, 0.1862,
#                                           0.1864, 0.1864, 0.1861, 0.1863, 0.1864 ]
# _batchPS3_asymmetric['results']['40'] = [ 0.2625,
#                                           0.2595, 0.2590, 0.2588, 0.2588, 0.2590,
#                                           0.2591, 0.2588, 0.2591, 0.2590, 0.2591,
#                                           0.2590, 0.2590, 0.2590, 0.2588, 0.2588,
#                                           0.2590, 0.2590, 0.2592, 0.2589, 0.2588 ]
# _batchPS3_asymmetric['results']['50'] = [ 0.2685,
#                                           0.2697, 0.2699, 0.2700, 0.2698, 0.2697,
#                                           0.2697, 0.2699, 0.2696, 0.2695, 0.2698,
#                                           0.2697, 0.2700, 0.2696, 0.2699, 0.2697,
#                                           0.2696, 0.2700, 0.2699, 0.2697, 0.2699 ]


# _batchPS3_rings = {}
# _batchPS3_rings['pattern'] = 'rings'
# _batchPS3_rings['results'] = {}
# _batchPS3_rings['results']['20'] = [ 0.1991, 
#                                      0.1937, 0.1929, 0.1939, 0.1939, 0.1928,
#                                      0.1912, 0.1921, 0.1935, 0.1924, 0.1926,
#                                      0.1931, 0.1921, 0.1938, 0.1935, 0.1924, 
#                                      0.1935, 0.1930, 0.1914, 0.1921, 0.1931 ]
# _batchPS3_rings['results']['30'] = [ 0.2112,
#                                      0.2066, 0.2079, 0.2080, 0.2076, 0.2078,
#                                      0.2085, 0.2078, 0.2080, 0.2083, 0.2077,
#                                      0.2077, 0.2066, 0.2081, 0.2074, 0.2077, 
#                                      0.2071, 0.2073, 0.2079, 0.2074, 0.2083 ]
# _batchPS3_rings['results']['40'] = [ 0.2966,
#                                      0.2806, 0.2820, 0.2816, 0.2819, 0.2821,
#                                      0.2824, 0.2818, 0.2815, 0.2816, 0.2825,
#                                      0.2817, 0.2821, 0.2820, 0.2822, 0.2820,
#                                      0.2819, 0.2820, 0.2823, 0.2820, 0.2823 ]
# _batchPS3_rings['results']['50'] = [ 0.2425,
#                                      0.2425, 0.2475, 0.2401, 0.2415, 0.2470, 
#                                      0.2409, 0.2400, 0.2433, 0.2482, 0.2541,
#                                      0.2412, 0.2427, 0.2458, 0.2403, 0.2400,
#                                      0.2405, 0.2399, 0.2408, 0.2401, 0.2401 ]


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