from geo_parser import GeoParser
from final_pathfinding import Pathfinder
from executelist_parser import ExecuteListParser

user_data = {
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": {
        "stroke": "#000000",
        "stroke-width": 2,
        "stroke-opacity": 1,
        "fill": "#555555",
        "fill-opacity": 0.5
      },
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [
              -3.199559805791,
              55.9380467435949
            ],
            [
              -3.1979541336915016,
              55.9380467435949
            ],
            [
              -3.1979541336915016,
              55.938946063958625
            ],
            [
              -3.199559805791,
              55.938946063958625
            ],
            [
              -3.199559805791,
              55.9380467435949
            ]
          ]
        ]
      },
      "id": "5c386581-e00b-4b2e-9e34-41c7974b4b16"
    },
    {
      "type": "Feature",
      "properties": {
        "stroke": "#ff0000",
        "stroke-width": 2,
        "stroke-opacity": 1
      },
      "geometry": {
        "type": "LineString",
        "coordinates": [
          [
            -3.1985262036323547,
            55.93840607393455
          ],
          [
            -3.198853433132171,
            55.938295657531725
          ]
        ]
      }
    },
    {
      "type": "Feature",
      "properties": {
        "stroke": "#0000ff",
        "stroke-width": 2,
        "stroke-opacity": 1,
        "fill": "#555555",
        "fill-opacity": 0.5
      },
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [
              -3.198595941066742,
              55.93861338551411
            ],
            [
              -3.198350518941879,
              55.93861338551411
            ],
            [
              -3.198350518941879,
              55.93878839768681
            ],
            [
              -3.198595941066742,
              55.93878839768681
            ],
            [
              -3.198595941066742,
              55.93861338551411
            ]
          ]
        ]
      }
    },
    {
      "type": "Feature",
      "properties": {
        "stroke": "#ff0000",
        "stroke-width": 2,
        "stroke-opacity": 1
      },
      "geometry": {
        "type": "LineString",
        "coordinates": [
          [
            -3.1988346576690674,
            55.93866971999075
          ],
          [
            -3.1989151239395137,
            55.938532263723815
          ]
        ]
      }
    },
    {
      "type": "Feature",
      "properties": {
        "stroke": "#000000",
        "stroke-width": 2,
        "stroke-opacity": 1,
        "fill": "#000000",
        "fill-opacity": 0.5
      },
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [
              -3.200362642,
              55.937597084
            ],
            [
              -3.197151298,
              55.937597084
            ],
            [
              -3.197151298,
              55.939395724
            ],
            [
              -3.200362642,
              55.939395724
            ],
            [
              -3.200362642,
              55.937597084
            ]
          ]
        ]
      }
    }
  ]
}


geo_parser = GeoParser()
drawing_points = geo_parser.parser_file_red(user_data)
obsolete_list = geo_parser.parser_file_blue(user_data)
print('drawing_points=%s}' % (drawing_points))
print('obsolete_list=%s}' % (obsolete_list))
start_position = [-3.200362642, 55.937597084]
point_finder = Pathfinder(start_position, drawing_points, obsolete_list)
point_list = point_finder.all_together_now()
print('Pointlist=%s}' % (point_list))
execute_list = ExecuteListParser(point_list)
print('Job planning execute list success execute_list=%s}' % (execute_list))
