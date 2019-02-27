#pragma once
#include <list>
#include <Geometry/Geometry.h>
#include <Geometry/BoundingBox.h>
#include <memory>
#include <algorithm>
#include <omp.h>  

#include <cmath>


class BVH_SAH
{
private:

	class Node {
	private:
		Geometry::BoundingBox m_boundingbox;
		std::shared_ptr<Node> fils_droit;
		std::shared_ptr<Node> fils_gauche;
		::std::deque<Geometry::Triangle const *> m_triangles;

	public:
		//Constructeur
		Node(::std::deque<const Geometry::Triangle *> triangles) {

			//Calcul de la BoundingBox
			Math::Vector3f min(triangles.front()->vertex(0));
			Math::Vector3f max(triangles.front()->vertex(0));

			for (Geometry::Triangle const * tri : triangles) {
				for (int i = 0; i < 3; i++) {

					min = min.simdMin(tri->vertex(i));
					max = max.simdMax(tri->vertex(i));
				}
			}
			m_boundingbox = Geometry::BoundingBox(min, max);

			if (triangles.size() <8 ) {
				m_triangles = triangles;

			}
			else {

				//Initialisation des liste de retour
				::std::deque<Geometry::Triangle const *> list_fils_Gauche;
				::std::deque<Geometry::Triangle const *>  list_fils_Droit;



				//////////////////////////////////////////////////////////
				///////////////////// CHOIX DE COUPE /////////////////////
				//////////////////////////////////////////////////////////

				//Calcul de l'axe le plus long
				int axeMin = longAxe(min, max);
				double cout = 0;
				double cout_min = std::numeric_limits<double>::max();
				double index_cout_min = 0;
				unsigned int i;
				double traversal_cost = 1.;
				double intersection_cost = 1.;
				double box_area = calculAire(getBB());
				const unsigned int size = triangles.size();

				//calcul de la meilleure coupe selon les 3 axes
				for (int axe = 0; axe < 3; axe++) {
					
					//comparateur de position relative de 2 triangles selon 1 axe
					auto comparateur = [axe](const Geometry::Triangle *  t1, const Geometry::Triangle *  t2) {
						return calculPosition(t1, axe) < calculPosition(t2, axe);
					};

					::std::sort(triangles.begin(), triangles.end(), comparateur);
					// création de 2 listes de Bounding Box pour la création de toutes les possibilités de coupure.
					Geometry::BoundingBox * bb_droit = new Geometry::BoundingBox[size + 1];
					Geometry::BoundingBox * bb_gauche = new Geometry::BoundingBox[size + 1];

					for (i = 1; i < size + 1; i++) {
						bb_gauche[i] = bb_gauche[i - 1];
						bb_gauche[i].update(*triangles[i - 1]);
						bb_droit[size - i] = bb_droit[size - i + 1];
						bb_droit[size - i].update(*triangles[size - i]);
					}


					for (i = 1; i < size; i++) {
						double aire_droit = calculAire(bb_droit[i]);
						//std::cout << "Aire droite : "<<aire_droit << std::endl;
						double aire_gauche = calculAire(bb_gauche[i]);
						//std::cout << "Aire gauche : " << aire_gauche << std::endl;
						cout = traversal_cost
							+ (aire_gauche / box_area)*i*intersection_cost
							+ (aire_droit / box_area)*(size + 1 - i)*intersection_cost;
						// évaluation du cout minimale
						if (cout < cout_min) {
							cout_min = cout;
							index_cout_min = i;
							axeMin = axe;
						}
					}

					delete[] bb_gauche;
					delete[] bb_droit;
				}

				auto comparateur = [axeMin](const Geometry::Triangle *  t1, const Geometry::Triangle *  t2) {
					return calculPosition(t1, axeMin) < calculPosition(t2, axeMin);
				};

				::std::sort(triangles.begin(), triangles.end(), comparateur);

				for (unsigned int j = 0; j < size; j++) {
					if (j < index_cout_min) {
						list_fils_Droit.push_back(triangles[j]);
					}
					else {
						list_fils_Gauche.push_back(triangles[j]);
					}
				}

				triangles.clear();
				

				//appel récursif de la création de l'arbre
				fils_droit = std::shared_ptr<Node>(new Node(list_fils_Droit));
					
					 
				fils_gauche = std::shared_ptr<Node>(new Node(list_fils_Gauche));
					
			}
		};

		Geometry::BoundingBox const & getBB() const {
			return m_boundingbox;
		}

		bool isALeaf() {
			return m_triangles.size() != 0;
		}

		::std::deque<Geometry::Triangle const *> & listTriangle() {
			return m_triangles;
		}

		std::shared_ptr<Node> & childRight() {
			return fils_droit;
		}
		std::shared_ptr<Node> & childLeft() {
			return fils_gauche;
		}

		~Node() {};
	};



	std::shared_ptr<Node> m_sommet;

public:
	BVH_SAH();
	bool intersect(Geometry::Ray const & ray, Geometry::RayTriangleIntersection& intersection);
	bool intersect(Geometry::Ray const & ray, Geometry::RayTriangleIntersection & intersection, std::shared_ptr<Node> node);
	void init(::std::deque<::std::pair<Geometry::BoundingBox, Geometry::Geometry> >* geometries);
	virtual ~BVH_SAH();

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////// Methode static personnel ///////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	/*
	*	Rend la valeur médiane d'une liste
	*
	*/
	double static med_val(std::list<double> position) {
		int size = position.size();
		int i = 0;
		for (double a : position) {
			if (i != size / 2) {
				++i;
			}
			else {
				return a;
			}
		}
		return 0.;
	}

	/*
	*	Renvoie l'axe le plus long sur une BB désigné par 2 points
	*
	*/
	int static longAxe(Math::Vector3f min, Math::Vector3f max) {
		double X = max[0] - min[0];
		double Y = max[1] - min[1];
		double Z = max[2] - min[2];
		if (X > Y) {
			if (X > Z) {
				return 0;
			}
			else {
				return 2;
			}
		}
		else {
			if (Y > Z) {
				return 1;
			}
			else {
				return 2;
			}
		}
	}

	/*
	* Calcul l'aire d'une BB
	*/
	double static calculAire(Geometry::BoundingBox const & bb) {
		double aire = (bb.max()[0] - bb.min()[0])*(bb.max()[1] - bb.min()[1]) * 2;
		aire += (bb.max()[2] - bb.min()[2])*(bb.max()[1] - bb.min()[1]) * 2;
		aire += (bb.max()[0] - bb.min()[0])*(bb.max()[2] - bb.min()[2]) * 2;
		return sqrt(aire);
	}

	/*
	* Calcul d'une position sur un axe à partir d'un triangle pour trie
	*/
	double static calculPosition(const Geometry::Triangle * tri, int axe) {
		bool mode = true; //true if moyen, false for center
		if (mode) {
			double result = 0;
			for (int i = 0; i < 3; ++i) {
				result += tri->vertex(i)[axe];
			}
			result /= 3;
			return result;
		}
		else {
			return tri->center()[axe];
		}

	}
};


