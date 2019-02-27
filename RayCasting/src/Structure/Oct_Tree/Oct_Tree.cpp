#include <Structure/Oct_Tree.h>

namespace Geometry {

	Oct_Tree::Oct_Tree()
	{
	}

	bool Oct_Tree::intersect(Ray const & ray, RayTriangleIntersection & intersection)
	{
		return false;
	}

	void Oct_Tree::init(::std::deque<::std::pair<BoundingBox, Geometry> >* geometries)
	{
	}


	Oct_Tree::~Oct_Tree()
	{
	}
}