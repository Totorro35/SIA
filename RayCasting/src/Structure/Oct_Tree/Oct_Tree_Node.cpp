#include "Oct_Tree_Node.h"

Oct_Tree_Node::Oct_Tree_Node()
{
}

void Oct_Tree_Node::init(::std::deque<::std::pair<Geometry::BoundingBox, Geometry::Geometry>>& geometries)
{

	for (::std::pair<Geometry::BoundingBox, Geometry::Geometry > & geometrie : geometries) {

	}
}

Oct_Tree_Node::~Oct_Tree_Node()
{
	delete* m_branch;
}
