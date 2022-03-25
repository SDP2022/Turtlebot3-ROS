### Pyhton script that takes in the geojson file and outputs array of paris of nodes.
### Outputes pairs of nodes indicating red lines that are to be painted
## Blue lines are objects needed to be avoided.


import json

class GeoParser(object):
    def parser_file_red(file_path):
        with open(file_path) as f:
            gj = json.load(f)

        red_points = []
        no_features = len(gj['features'])

        for x in range(no_features):
            features = gj['features'][x]
            if features['properties']['stroke'] == '#ff0000':
                point_one = (features['geometry']['coordinates'])[0]
                point_two = (features['geometry']['coordinates'])[1]
                no_of_points = 2 ##len(points)
                ##for t in range(no_of_points):
                red_points.append((point_one, point_two))

        return red_points


    def parser_file_blue(file_path):
        with open(file_path) as f:
            gj = json.load(f)

        blue_list = []
        no_features = len(gj['features'])

        for x in range(no_features):
            features = gj['features'][x]
            if features['properties']['stroke'] == '#0000ff':
                blue_list = []
                no_of_points = len((features['geometry']['coordinates'])[0])

                for t in range(no_of_points):
                    point = (features['geometry']['coordinates'])[0]
                    blue_list.append(point[0])
        return blue_list
