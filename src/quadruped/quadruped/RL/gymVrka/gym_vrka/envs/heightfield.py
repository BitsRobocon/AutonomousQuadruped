""" 
Implemented by using opensimplex and pybullet
"""
import math
import random
import pybullet as p
import pybullet_data as pd
import time

from opensimplex import OpenSimplex

tmp = OpenSimplex()

## get these values as input when executing
style = 'simplex'
octaves = 4
amplitude = 1
exponent = 0.25

numHeightfieldRows = 256
numHeightfieldColumns = 256

# def all_random(x, y):
#     return random.uniform(0, 0.05) # some error to correct later

class HeightField():
    def __init__(self):
        self.heightfieldData = [0] * numHeightfieldRows * numHeightfieldColumns

    def multiple_octaves(octaves, start_amplitude):
        parameters = []
        for i in range(octaves):
            parameters.append({
                'offset': random.random() * 2 * math.pi,
                'frequency': 2**i,
                'amplitude': start_amplitude / float(i+1),
        })

        def noise(x, y):
            value = 0
            for p in parameters:
                x_part = math.sin(
                    (x / float(numHeightfieldRows))
                    * p['frequency']
                    * 2 * math.pi
                    + p['offset']
                )
                y_part = math.sin(
                    (y / float(numHeightfieldColumns))
                    * p['frequency']
                    * 2 * math.pi
                    + (x_part % (2 * math.pi))
                )
                value += y_part * p['amplitude']

            return value

        return noise


    power_simplex = OpenSimplex(int(random.random() * 100))


    def power(exponent):
        def noise(x, y):
            value = (HeightField.power_simplex.noise2d(x/3.0, y/3.0) + 1.0) / 2.0

            return (value ** exponent) * 1.0

        return noise


    def simplex():
        tmp = OpenSimplex(int(random.random() * 10000))

        def noise(x, y):
            return (tmp.noise2d(x/3.0, y/3.0) + 1) * 0.05

        return noise


    def simple_curve(value):
        start = 0.4
        end = 0.6
        if value < start:
            return 0.0
        if value > end:
            return 1.0
        return (value - start) * (1 / (end - start))


    def interpolate(a, b, weight):
        new_weight = HeightField.simple_curve(weight)

        return a * (1 - new_weight) + b * new_weight


    def simple_scurve():
        tmp = OpenSimplex(int(random.random() * 10000))

        def noise(x, y):
            noise = (tmp.noise2d(x/5.0, y/5.0) + 1) / 2.0

            return HeightField.interpolate(0.0, 1.0, noise) * .25

        return noise


    def plains():
        tmp = OpenSimplex(int(random.random() * 1000))

        def noise(x, y):
            value = (tmp.noise2d(x/3.0, y/3.0) + 1) / 2.0

            value = value**0.25

            value = value - 0.6

            if value < 0:
                value = 0

            return value * .50

        return noise


    def mountains():
        tmp = OpenSimplex(int(random.random() * 10000))

        def noise(x, y):
            value = (tmp.noise2d(x*2.0, y) + 1) / 2.0

            value = value

            return value * .50

        return noise


    def combined():
        m_values = HeightField.mountains()
        p_values = HeightField.plains()
        weights = HeightField.simple_scurve()

        def noise(x, y):
            m = m_values(x, y)
            p = p_values(x, y)
            w = weights(x, y) / 10.0

            return (p * w) + (m * (1 - w))

        return noise


    def generate(value):
        values = []

        for x in range(numHeightfieldRows):
            for y in range(numHeightfieldColumns):
                values.append(value(x, y))

        return values


# if __name__ == "__main__":

#     hf = HeightField
#     value_func = hf.simplex()

#     if style == 'octaves':
#         # get octaves
#         value_func = hf.multiple_octaves(octaves, amplitude)
#     elif style == 'simplex':
#         value_func = hf.simplex()
#     elif style == 'power':
#         # get exponent
#         value_func = hf.power(exponent)
#     elif style == 'scurve':
#         value_func = hf.simple_scurve()
#     elif style == 'plains':
#         value_func = hf.plains()
#     elif style == 'mountains':
#         value_func = hf.mountains()  # to be fixed later use octaves instead
#     elif style == 'combined':
#         value_func = hf.combined()

#     heightfieldData = hf.generate(value_func)

#     terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.05,.05,1], heightfieldTextureScaling=(numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
#     terrain  = p.createMultiBody(0, terrainShape)
#     p.resetBasePositionAndOrientation(terrain,[0,0,0], [0,0,0,1])
#     p.changeVisualShape(terrain, -1, rgbaColor=[1,1,1,1])

#     p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
#     p.setGravity(0, 0, -10)
#     p.setRealTimeSimulation(1)

#     while (p.isConnected()):
#       keys = p.getKeyboardEvents()
#       time.sleep(0.1)


def UpdateHeightField(self):
    #GEOM_CONCAVE_INTERNAL_EDGE may help avoid getting stuck at an internal (shared) edge of the triangle/heightfield.
    #GEOM_CONCAVE_INTERNAL_EDGE is a bit slower to build though.
    #flags = p.GEOM_CONCAVE_INTERNAL_EDGE

    # env.pybullet_client.setAdditionalSearchPath(pd.getDataPath())
    # env.pybullet_client.configureDebugVisualizer(env.pybullet_client.COV_ENABLE_RENDERING, 0)


    value_func = self.simplex()

    if style == 'octaves':
        # get octaves
        value_func = self.multiple_octaves(octaves, amplitude)
    elif style == 'simplex':
        value_func = self.simplex()
    elif style == 'power':
        # get exponent
        value_func = self.power(exponent)
    elif style == 'scurve':
        value_func = self.simple_scurve()
    elif style == 'plains':
        value_func = self.plains()
    elif style == 'mountains':
        value_func = self.mountains()  # to be fixed later use octaves instead
    elif style == 'combined':
        value_func = self.combined()

    self.heightfieldData = self.generate(value_func)
    
    flags = 0
    self.terrainShape = p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                flags=flags,
                meshScale=[.05, .05, 1],
                heightfieldTextureScaling=(numHeightfieldRows - 1) / 2,
                heightfieldData=self.heightfieldData,
                numHeightfieldRows=numHeightfieldRows,
                numHeightfieldColumns=numHeightfieldColumns)