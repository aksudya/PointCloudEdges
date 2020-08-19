#pragma once
#include "polyscope/polyscope.h"

#include "polyscope/combining_hash_functions.h"
#include "polyscope/messages.h"

#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/surface_mesh_io.h"
#include "polyscope/file_helpers.h"


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/point_generators_2.h>
//#include <CGAL/Optimal_transportation_reconstruction_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_3.h>
#include <queue>
#include <CGAL/Point_set_3.h>
#include <list>
#include <CGAL/Polygon_2.h>
#include <CGAL/Periodic_3_triangulation_3.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Optimal_transportation_reconstruction_2.h>
//#include <gl/GLUT.h>
#include <iostream>
#include <cmath>
#include <random>
#include <ctime>
#include <fstream> 
#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <list>

#include <iostream>
#include <unordered_set>
#include <utility>

#include "args/args.hxx"
#include "json/json.hpp"
#include "nanoflann.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/string_cast.hpp"

#include "stb_image.h"


using std::cerr;
using std::cout;
using std::endl;
using std::string;

using namespace std;
using namespace nanoflann;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef K::Point_3	                                        Point;
typedef CGAL::Delaunay_triangulation_3<K>					Delaunay;
typedef CGAL::Triangulation_3<K>							Triangulation;
typedef Triangulation::Edge									Edge;

typedef Delaunay::Vertex_handle								d_vertex_handle;
typedef Triangulation::Vertex_handle						vertex_handle;
typedef K::Segment_3										Segment;
typedef CGAL::Polygon_2<K>									Polygon_2;
typedef K::Line_3											Line;
typedef pair<vertex_handle, vertex_handle>					vertex_pair;

typedef K::Vector_3											Vector;
