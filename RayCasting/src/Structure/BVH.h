#pragma once
#include <list>
#include <Geometry/Geometry.h>
#include <Geometry/BoundingBox.h>
#include <memory>

#include <cmath>


class BVH
{
	private:

		class Node {
			private :
				Geometry::BoundingBox m_boundingbox;
				std::shared_ptr<Node> fils_droit;
				std::shared_ptr<Node> fils_gauche;
				::std::list<Geometry::Triangle const *> m_triangles;

			public :
				//Constructeur
				Node(::std::list<Geometry::Triangle const*> triangles) {
					//::std::cout << "Nouveau noeud" << std::endl;
					//::std::cout << triangles.size() << std::endl;

					//Calcul de la BoundingBox
					Math::Vector3f min(triangles.front()->vertex(0));
					Math::Vector3f max(triangles.front()->vertex(0));

					for (Geometry::Triangle const * tri : triangles) {
						for (int i = 0; i < 3; i++) {

							min = min.simdMin(tri->vertex(i));
							max = max.simdMax(tri->vertex(i));
						}
					}
					//calcul de la bounding Box
					m_boundingbox = Geometry::BoundingBox(min, max);

					if (triangles.size() < 50) {
						//::std::cout << "C'est une feuille" << std::endl;
						//Calcul de la BoundingBox
						m_triangles = triangles;

					}else {
						//::std::cout << "		noeud" << std::endl;

						//Initialisation des liste de retour
						::std::list<Geometry::Triangle const *> list_fils_Gauche;
						::std::list<Geometry::Triangle const *>  list_fils_Droit;



						//////////////////////////////////////////////////////////
						///////////////////// CHOIX DE COUPE /////////////////////
						//////////////////////////////////////////////////////////
						
						//Calcul de l'axe le plus long
						int axe = longAxe(min, max);
						double separator = 0.;

						
						
							//Calcul du séparateur selon la médiane
							::std::list<double> position;

							for (Geometry::Triangle const * tri : triangles) {
								position.push_back(tri->center()[axe]);// calcul du centre du triangle selon l'axe étudié
							}
							position.sort();


							separator = med_val(position);
						


						//////////////////////////////////////////////////////////////////

						// Répartition selon separator sur l'axe le plus long

						for (Geometry::Triangle const * tri : triangles) {
							double point = calculPosition(tri, axe);
							if (point > separator) {
								list_fils_Droit.push_back(tri);
							}
							else if(point < separator){
								list_fils_Gauche.push_back(tri);
							}
							else {
								if (list_fils_Droit.size() < list_fils_Gauche.size()) {
									list_fils_Droit.push_back(tri);
								}
								else {
									list_fils_Gauche.push_back(tri);
								}
							}
							
						}

						//Set des listes sur les fils et appel récursif de la création de l'arbre
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

				::std::list<Geometry::Triangle const *> & listTriangle() {
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
		BVH();
		bool intersect(Geometry::Ray const & ray, Geometry::RayTriangleIntersection& intersection);
		bool intersect(Geometry::Ray const & ray, Geometry::RayTriangleIntersection & intersection, std::shared_ptr<Node> node);
		void init(::std::deque<::std::pair<Geometry::BoundingBox, Geometry::Geometry> >* geometries);
		virtual ~BVH();

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
		double static calculPosition(Geometry::Triangle const * tri, int axe) {
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


