#include "BVH.h"



BVH::BVH()
{
}

bool BVH::intersect(Geometry::Ray const & ray, Geometry::RayTriangleIntersection & intersection)
{
	return intersect(ray,intersection,m_sommet);
}

bool BVH::intersect(Geometry::Ray const & ray, Geometry::RayTriangleIntersection & intersection, std::shared_ptr<Node> node) {

	double t1(std::numeric_limits<double>::max());
	double entry;
	double exit;
	//calcul d'intersection avec la BoundingBox
	if (node->getBB().intersect(ray, 0., t1, entry, exit)) {

		Geometry::CastedRay casted_ray(ray);
		if (node->isALeaf()) {

			for (Geometry::Triangle const * tri : node->listTriangle()) {
				casted_ray.intersect(tri);
			}
			bool result = casted_ray.validIntersectionFound();
			if (result) {
				intersection = casted_ray.intersectionFound();
				return true;
			}
		}
		else {
			Geometry::RayTriangleIntersection intersection_tmp;
			//Test d'intersection recursif sur le fils droit si BB toucher!
			if (this->intersect(ray, intersection_tmp, node->childRight())) {
				casted_ray.intersect(intersection_tmp.triangle());
			}
			//Test d'intersection recursif sur le fils droit si BB toucher!
			if (this->intersect(ray, intersection_tmp, node->childLeft())) {
				casted_ray.intersect(intersection_tmp.triangle());
			}
			bool result = casted_ray.validIntersectionFound();
			if (result) {
				intersection = casted_ray.intersectionFound();
				return true;
			}
		}
	}
	return false;
}

void BVH::init(::std::deque<::std::pair<Geometry::BoundingBox, Geometry::Geometry>>* geometries)
{
	::std::list<Geometry::Triangle const *> list_triangle;
	for (::std::pair<Geometry::BoundingBox, Geometry::Geometry>& element : *geometries) {
		for (Geometry::Triangle const & triangle : element.second.getTriangles()) {
			list_triangle.push_back(&triangle);
		}
	}
	//appel de la méthode de construction récursive de l'arbre
	m_sommet = std::shared_ptr<Node>(new Node(list_triangle));
	std::cout << "Construction du BVH terminee" << std::endl;
}


BVH::~BVH()
{
}
