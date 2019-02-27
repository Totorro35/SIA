#include "Box_geometry.h"

namespace Geometry
{
	Box_geometry::Box_geometry()
	{

	}

	bool Box_geometry::intersect(Ray const & ray, RayTriangleIntersection & intersection)
	{
		CastedRay casted_ray(ray);

		double t1(std::numeric_limits<double>::max());

		for (std::pair < BoundingBox, ::Geometry::Geometry > & element : *m_geometries) {

			double entry;
			double exit;
			if(element.first.intersect(ray, 0., t1, entry, exit)) 
			{
				if (element.second.intersection(casted_ray)) {
					t1 = exit;
				}
			}
		}

		if (casted_ray.validIntersectionFound()) {
			intersection = casted_ray.intersectionFound();
			return true;
		}
		else {
			return false;
		}
	}

	void Box_geometry::init(::std::deque<::std::pair<BoundingBox, Geometry> >* geometries)
	{
		this->m_geometries = geometries;
	}


	Box_geometry::~Box_geometry()
	{

	}
}
