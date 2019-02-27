#pragma once
#include <queue>
#include <Geometry/Geometry.h>
#include <Geometry/BoundingBox.h>

namespace Geometry
{
	class Box_geometry
	{

	private:
		::std::deque<::std::pair<BoundingBox, Geometry> >* m_geometries;

	public:
		Box_geometry();
		bool intersect(Ray const & ray, RayTriangleIntersection& intersection);
		void init(::std::deque<::std::pair<BoundingBox, Geometry> >* geometries);

		virtual ~Box_geometry();
	};
}

