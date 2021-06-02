#include <algorithm>
#include "software/geom/algorithms/rasterize.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/algorithms/contains.h"


bool isInPixel(const Point &a, const Point &b, double resolution_size);

bool isAVertex(const Point& point, const Polygon& polygon, double resolution_size);

// TODO When rasterizing without knowing the relative positions of the pixels, you may be off by 1 pixel in each
// axis. eg. A 1.5 x 1 rectangle may overlap with 2 or 3 pixels (assuming pixel dimension 1) depending on how it the rectangle
// is positioned.
std::vector<Point> rasterize(const Circle &circle, const double resolution_size)
{
    std::vector<Point> covered_points;

    Point origin = circle.origin();
    double radius = circle.radius();

    // Added resolution_size to the max to ensure that when Points are rasterized, all
    // Coordinates (including pixels that are not fully contained by rectangle) are
    // taken into account.
    for (double x_offset = -radius; x_offset < radius + resolution_size; x_offset += resolution_size)
    {
        for (double y_offset = -radius; y_offset < radius + resolution_size; y_offset += resolution_size)
        {
            if(x_offset * x_offset + y_offset * y_offset < radius * radius)
            {
                covered_points.emplace_back(Point(origin.x() + x_offset, origin.y() + y_offset));
            }
        }
    }
    return covered_points;

//    std::set<Point> points_covered;
//
//    int radius              = circle.radius();
//    Point center_point      = circle.origin();
//    double xc = center_point.x();
//    double yc = center_point.y();
//
//    int x = 0, y = radius;
//    int d = 3 - 2 * radius;
//
//    auto set_blocked_coordinates = [points_covered, xc, yc](double x, double y){
//        points_covered.insert(Point(xc+x, yc+y));
//        points_covered.insert(Point(xc-x, yc+y));
//        points_covered.insert(Point(xc+x, yc-y));
//        points_covered.insert(Point(xc-x, yc-y));
//        points_covered.insert(Point(xc+y, yc+x));
//        points_covered.insert(Point(xc-y, yc+x));
//        points_covered.insert(Point(xc+y, yc-x));
//        points_covered.insert(Point(xc-y, yc-x));
//    };
//
//    set_blocked_coordinates(x, y);
//
//    while (y >= x)
//    {
//        x++;
//
//        if (d > 0)
//        {
//            y--;
//            d = d + 4 * (x - y) + 10;
//        }
//        else
//        {
//            d = d + 4 * x + 6;
//        }
//        set_blocked_coordinates(x, y);
//    }

    return std::vector<Point>();
}

std::vector<Point> rasterize(const Rectangle &rectangle, const double resolution_size)
{
    std::vector<Point> covered_points;

    // Added resolution_size to the max to ensure that when Points are rasterized, all
    // Coordinates (including pixels that are not fully contained by rectangle) are
    // taken into account.
    for (double x = rectangle.xMin(); x < rectangle.xMax() + resolution_size; x += resolution_size)
    {
        for (double y = rectangle.yMin(); y < rectangle.yMax() + resolution_size; y += resolution_size)
        {
            covered_points.emplace_back(Point(x, y));
        }
    }
    return covered_points;
}

std::vector<Point> rasterize(const Polygon &polygon, const double resolution_size)
{
    // Using even-odd rule algorithm to fill in polygon
    // https://stackoverflow.com/a/31768384

    std::vector<Point> contained_points;
	const auto& polygon_vertices = polygon.getPoints();

    auto max_point_y = [](const Point& a, const Point& b) {
       return a.y() < b.y();
    };

    auto max_point_x = [](const Point& a, const Point& b) {
       return a.x() < b.x();
    };

    // Calculate the highest and lowest x and y points
    double min_y  = std::min_element(polygon_vertices.begin(), polygon_vertices.end(), max_point_y)->y();
    double min_x  = std::min_element(polygon_vertices.begin(), polygon_vertices.end(), max_point_x)->x();
	double max_y  = std::max_element(polygon_vertices.begin(), polygon_vertices.end(), max_point_y)->y();

    //loop through rows of the image (i.e. polygon)
    for (double y_coord = min_y; y_coord <= max_y; y_coord += resolution_size)
    {
        //we create a line that intersects the polygon at this y coordinate
        Ray intersecting_ray = Ray(Point(min_x, y_coord), Vector(1, 0));

		auto intersections_with_polygon = intersection(polygon, intersecting_ray);
		std::vector<Point> sorted_intersections_with_polygon(intersections_with_polygon.begin(),
															 intersections_with_polygon.end());
		std::sort(sorted_intersections_with_polygon.begin(), sorted_intersections_with_polygon.end(),
				  max_point_x);

		auto num_of_intersections = sorted_intersections_with_polygon.size();
		unsigned int intersection_index = 0;
		double x_coord = min_x;
		bool in_polygon = false;

		while (intersection_index < num_of_intersections)
		{
			Point point = Point(x_coord, y_coord);
			bool isCloseToIntersectionPoint = isInPixel(point,
														sorted_intersections_with_polygon[intersection_index],
														resolution_size);
			if (isCloseToIntersectionPoint && !isAVertex(point, polygon, resolution_size))
			{
				in_polygon = !in_polygon;
				intersection_index++;
			}
			else if (isCloseToIntersectionPoint)
			{
				intersection_index++;
			}

			if (isAVertex(point, polygon, resolution_size) || in_polygon)
			{
				contained_points.emplace_back(point);
			}

			x_coord += resolution_size;
		}
    }

//    std::vector<double> node_x;
//    const auto& points = polygon.getPoints();
//
//    auto max_point_y = [](const Point& a, const Point& b) {
//        return a.y() < b.y();
//    };
//    auto max_point_x = [](const Point& a, const Point& b) {
//        return a.x() < b.x();
//    };
//
//    // Calculate the highest and lowest x and y points
//    double max_y = std::max_element(points.begin(), points.end(), max_point_y)->y();
//    double min_y  = std::min_element(points.begin(), points.end(), max_point_y)->y();
//    double max_x = std::max_element(points.begin(), points.end(), max_point_x)->x();
//    double min_x  = std::min_element(points.begin(), points.end(), max_point_x)->x();
//
//    int num_edges = static_cast<int>(points.size());
//
//    // Loop through the rows of the image (i.e. polygon)
//    for (double pixel_y = min_y; pixel_y < max_y; pixel_y += resolution_size) {
//        int nodes = 0;
//        int j = num_edges - 1;
//        for (int i = 0; i < num_edges; i++) {
//            if ((points[i].y() < pixel_y && points[j].y() >= pixel_y) ||
//                (points[j].y() < pixel_y && points[i].y() >= pixel_y)) {
//                // TODO Changed cast
//                node_x[nodes++] = (points[i].x() +
//                                         ((double) (pixel_y - points[i].y()) / (points[j].y() - points[i].y())) *
//                                         (points[j].x() - points[i].x()));
//            }
//            j = i;
//        }
//
//        // TODO double check that the array is being sorted in correct order.
//        std::sort(node_x.begin(), node_x.end());
//
//        // Fill the pixels between node pairs
//        for (int i = 0; i < nodes; i += 2) {
//            if (node_x[i] >= max_x) break;
//            if (node_x[i + 1] > min_x) {
//                if (node_x[i] < min_x) node_x[i] = min_x;
//                if (node_x[i + 1] > max_x) node_x[i + 1] = max_x;
//                // TODO Casted to int?
//                for (j = (int) node_x[i]; j < node_x[i + 1]; j++) {
//                    contained_points.emplace_back(Point(j - min_x, pixel_y - min_y));
//                }
//            }
//
//        }
//    }

    return contained_points;
}

bool isInPixel(const Point &a, const Point &b, double resolution_size)
{
   	double min_x = a.x() - resolution_size / 2;
   	double min_y = a.y() - resolution_size / 2;
   	double max_x = a.x() + resolution_size / 2;
   	double max_y = a.y() + resolution_size / 2;

   	Rectangle pixel = Rectangle(Point(min_x, min_y), Point(max_x, max_y));
   	return contains(pixel, b);
}

bool isAVertex(const Point& point, const Polygon& polygon, double resolution_size)
{
	const auto& polygon_vertices = polygon.getPoints();
	for (auto i = polygon_vertices.begin(); i != polygon_vertices.end(); ++i)
   	{
	   	if (isInPixel(point, *i, resolution_size))
		{
			return true;
   		}
	}
	return false;
}
