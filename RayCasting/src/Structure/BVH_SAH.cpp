#include "BVH_SAH.h"



BVH_SAH::BVH_SAH()
{
}

bool BVH_SAH::intersect(Geometry::Ray const & ray, Geometry::RayTriangleIntersection & intersection)
{
	return intersect(ray, intersection, m_sommet);
}

bool BVH_SAH::intersect(Geometry::Ray const & ray, Geometry::RayTriangleIntersection & intersection, std::shared_ptr<Node> node) {

	

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
			double t1(std::numeric_limits<double>::max());

			bool bb1 = false;
			double entrybb1;
			double exitbb1;
			
			bool bb2 = false;
			double entrybb2;
			double exitbb2;

			if (node->childRight()->getBB().intersect(ray, 0., t1, entrybb1, exitbb1)) {
				bb1 = true;//Intersection trouvée avec la 1ere Bounding Box
			}
			if (node->childLeft()->getBB().intersect(ray, 0., t1, entrybb2, exitbb2)) {
				bb2 = true;//Intersection trouvée avec la 2eme Bounding Box
			}

			if (bb1) {
				if (bb2) {
					if (exitbb1<entrybb2) {// calcul des différents cas d'intersection possibles
						if (this->intersect(ray, intersection_tmp, node->childRight())) {
							casted_ray.intersect(intersection_tmp.triangle());
						}
						if (!casted_ray.validIntersectionFound()) {
							if (this->intersect(ray, intersection_tmp, node->childLeft())) {
								casted_ray.intersect(intersection_tmp.triangle());
							}
						}
					}
					else if (exitbb2<entrybb1){
						if (this->intersect(ray, intersection_tmp, node->childLeft())) {
							casted_ray.intersect(intersection_tmp.triangle());
						}
						if (!casted_ray.validIntersectionFound()) {
							if (this->intersect(ray, intersection_tmp, node->childRight())) {
								casted_ray.intersect(intersection_tmp.triangle());
							}
						}
					}
					else {
						//Test d'intersection recursif sur le fils droit si BB toucher!
						if (this->intersect(ray, intersection_tmp, node->childRight())) {
							casted_ray.intersect(intersection_tmp.triangle());
						}
						//Test d'intersection recursif sur le fils droit si BB toucher!
						if (this->intersect(ray, intersection_tmp, node->childLeft())) {
							casted_ray.intersect(intersection_tmp.triangle());
						}
					}
				}
				else {
					if (this->intersect(ray, intersection_tmp, node->childRight())) {
						casted_ray.intersect(intersection_tmp.triangle());
					}
				}
			}
			else {
				if (bb2) {
					if (this->intersect(ray, intersection_tmp, node->childLeft())) {
						casted_ray.intersect(intersection_tmp.triangle());
					}
				}
			}
			
			bool result = casted_ray.validIntersectionFound();
			if (result) {
				intersection = casted_ray.intersectionFound();
				return true;
			}
		}
	return false;
}


void BVH_SAH::init(::std::deque<::std::pair<Geometry::BoundingBox, Geometry::Geometry>>* geometries)
{
	//GExecutionTime Time;
	//Time.Start();

	
	::std::deque<Geometry::Triangle const *> list_triangle;
	for (::std::pair<Geometry::BoundingBox, Geometry::Geometry>& element : *geometries) {
		for (Geometry::Triangle const & triangle : element.second.getTriangles()) {
			list_triangle.push_back(&triangle);
		}
	}
	//appel récursif de construction de l'arbre
	m_sommet = std::shared_ptr<Node>(new Node(list_triangle));
	std::cout << "Construction du BVH terminee" << std::endl;

	//Time.Stop();
	//std::cout << "BVH en " << Time.Now() << " ms" << std::endl;
}


BVH_SAH::~BVH_SAH()
{
}
