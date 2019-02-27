#pragma once

#include <Geometry/Geometry.h>
#include <Geometry/BoundingBox.h>
#include <Geometry/Scene.h>
#include <Geometry/BoundingBox.h>
#include <queue>
#include <list>

namespace Geometry{

	class Oct_Tree
	{
		struct Element {
			BoundingBox m_bounding_box;
		};
		struct Node : Element {
			Oct_Tree::Element m_list[8];
		};
		struct Feuille : Element {
			::std::list<Triangle*> m_triangle;
		};

	private:
		Oct_Tree::Node m_sommet;

	public:
		Oct_Tree();
		bool intersect(Ray const & ray, RayTriangleIntersection& intersection);
		void init(::std::deque<::std::pair<BoundingBox, Geometry> >* geometries);
		virtual ~Oct_Tree();
	};
}
