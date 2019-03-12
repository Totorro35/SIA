#ifndef _Geometry_Scene_H
#define _Geometry_Scene_H

#include <windows.h>
#include <Geometry/Geometry.h>
#include <Geometry/PointLight.h>
#include <Visualizer/Visualizer.h>
#include <Geometry/Camera.h>
#include <Geometry/BoundingBox.h>
#include <Math/RandomDirection.h>
#include <windows.h>
#include <System/aligned_allocator.h>
#include <Math/Constant.h>
#include <queue>
#include <functional>
#include <random>
#include <Geometry/LightSampler.h>
#include <limits>
#include <Structure/Box_geometry.h>
#include <Structure/BVH_SAH.h>
#include <Structure/Oct_Tree.h>
#include <Structure/BVH.h>
#include <stdlib.h>
#include <iostream>
#include <Geometry/Texture.h>
#include <fstream>

namespace Geometry
{
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// \class	Scene
	///
	/// \brief	An instance of a geometric scene that can be rendered using ray casting. A set of methods
	/// 		allowing to add geometry, lights and a camera are provided. Scene rendering is achieved by
	/// 		calling the Scene::compute method.
	///
	/// \author	F. Lamarche, Université de Rennes 1
	/// \date	03/12/2013
	////////////////////////////////////////////////////////////////////////////////////////////////////
	class Scene
	{
	protected:
		/// \brief	The visualizer (rendering target).
		Visualizer::Visualizer * m_visu ;
		/// \brief	The scene geometry (basic representation without any optimization).
		::std::deque<::std::pair<BoundingBox, Geometry> > m_geometries ;
		//Geometry m_geometry ;
		/// \brief	The lights.
		std::vector<PointLight> m_lights ;
		/// \brief	The camera.
		Camera m_camera ;
		/// \brief The scene bounding box
		BoundingBox m_sceneBoundingBox;
		/// \brief number of diffuse samples for global illumination
		size_t m_diffuseSamples ;
		/// \brief Number of specular samples 
		size_t m_specularSamples ;
		/// \brief Number of light samples 
		size_t m_lightSamples;
		/// brief Rendering pass number
		int m_pass;

		std::string m_name;
		/// <summary>
		/// The light sampler associated with the scene
		/// </summary>
		std::vector < LightSampler> m_lightSamplers;

		///Structure d'optimisation
		//Box_geometry accelerator;
		BVH_SAH accelerator;
		//Oct_Tree accelerator; //Non implémenté
		//BVH accelerator;

		//Tableau Image
		::std::vector<::std::vector<::std::pair<int, RGBColor> > > pixelTable;


		double vitesse;
		Texture* skybox;

		double m_scoreLight;


		double energieLastPass = std::numeric_limits<double>::max();

		std::string energieString = "";
		std::string passCounter = "";
		

	public:

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	Scene::Scene(Visualizer::Visualizer * visu)
		///
		/// \brief	Constructor.
		///
		/// \author	F. Lamarche, Université de Rennes 1
		/// \date	03/12/2013
		///
		/// \param [in,out]	visu	If non-null, the visu.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		Scene(Visualizer::Visualizer * visu)
			: m_visu(visu), m_diffuseSamples(30), m_specularSamples(30), m_lightSamples(0),vitesse(2.),m_name("..\\..\\Resultat\\IlluminationGlobale\\")
		{}

		/// <summary>
		/// Prints stats about the geometry associated with the scene
		/// </summary>
		void printStats()
		{
			size_t nbTriangles = 0;
			for (auto it = m_geometries.begin(), end = m_geometries.end(); it != end; ++it)
			{
				nbTriangles += it->second.getTriangles().size();
			}
			::std::cout << "Scene: " << nbTriangles << " triangles" << ::std::endl;
		}

		/// <summary>
		/// Computes the scene bounding box.
		/// </summary>
		/// <returns></returns>
		const BoundingBox & getBoundingBox()
		{
			return m_sceneBoundingBox;
		}

		/// <summary>
		/// Computes the scene bounding box.
		/// </summary>
		/// <returns></returns>
		::std::deque<::std::pair<BoundingBox, Geometry> > & getAllGeometrie()
		{
			return m_geometries;
		}

		/// <summary>
		/// Sets the number of diffuse samples
		/// </summary>
		/// <param name="number"> The number of diffuse samples</param>
		void setDiffuseSamples(size_t number)
		{
			m_diffuseSamples = number;
		}

		/// <summary>
		/// Sets the number of specular samples
		/// </summary>
		/// <param name="number"></param>
		void setSpecularSamples(size_t number)
		{
			m_specularSamples = number;
		}

		/// <summary>
		/// Sets the number of light samples if the scene contains surface area lights
		/// </summary>
		/// <param name="number">The number of samples</param>
		void setLightSamples(size_t number)
		{
			m_lightSamples = number;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	void Scene::add(const Geometry & geometry)
		///
		/// \brief	Adds a geometry to the scene.
		///
		/// \author	F. Lamarche, Université de Rennes 1
		/// \date	03/12/2013
		///
		/// \param	geometry The geometry to add.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		void add(const Geometry & geometry)
		{
			if (geometry.getVertices().size() == 0) { return; }
			BoundingBox box(geometry) ;
			m_geometries.push_back(::std::make_pair(box, geometry)) ;
			m_geometries.back().second.computeVertexNormals(Math::piDiv4/2);
			if (m_geometries.size() == 1)
			{
				m_sceneBoundingBox = box;
			}
			else
			{
				m_sceneBoundingBox.update(box);
			}
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	void Scene::add(PointLight * light)
		///
		/// \brief	Adds a poitn light in the scene.
		///
		/// \author	F. Lamarche, Université de Rennes 1
		/// \date	04/12/2013
		///
		/// \param [in,out]	light	If non-null, the light to add.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		void add(const PointLight & light)
		{
			m_lights.push_back(light) ;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	void Scene::setCamera(Camera const & cam)
		///
		/// \brief	Sets the camera.
		///
		/// \author	F. Lamarche, Université de Rennes 1
		/// \date	04/12/2013
		///
		/// \param	cam	The camera.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		void setCamera(Camera const & cam)
		{
			m_camera = cam ;
		}



		void setName(std::string name)
		{
			m_name = name;
		}

		std::string getName() const
		{
			return m_name;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	Ray Scene::refract(Ray const & ray, RayTriangleIntersection& intersection)
		///
		/// \brief	Renvoie le rayon refracté issu du rayon incident et de l'intersection;
		///
		/// \author	S'tout mo vie
		/// \date	26/09/2018
		///
		/// \param	ray Le rayon lancé
		/// \param	intersection Le triangle intersecté avec le rayon
		/// \return	Le nouveau rayon calculé
		////////////////////////////////////////////////////////////////////////////////////////////////////
		Ray refract(Ray const & ray, RayTriangleIntersection& intersection) {
			Math::Vector3f normal = intersection.triangle()->sampleNormal(intersection.uTriangleValue(), intersection.vTriangleValue(), ray.source()); //Normal au triangle
			Ray result(ray.source(), normal*(-1));
			return result;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	double ombrage(Ray const & ray, RayTriangleIntersection& intersection)
		///
		/// \brief	Renvoie la valeur de l'ombre
		///
		/// \author	S. Corentin, L. Thomas
		/// \date	27/09/2018
		///
		/// \param	ray Le rayon lancé
		/// \param	intersection Le triangle intersecté avec le rayon
		/// \return	la valeur de l'ombrage associé
		////////////////////////////////////////////////////////////////////////////////////////////////////
		double ombrage(RayTriangleIntersection& intersection,PointLight const& light) {

			RayTriangleIntersection mur;
			Math::Vector3f light_dir = (intersection.intersection() - light.position()).normalized();
			
			Ray ray(light.position(), light_dir);
			accelerator.intersect(ray, mur);

			if (mur.valid()) {
				if (mur.triangle() == intersection.triangle()) {
					return 1.;
				}
			}

			return 0.;
		}

		PointLight samplingSphere(PointLight& light0) {
			double phi = acos(sqrt(Math::RandomDirection::random()));
			double theta = 2*Math::pi*Math::RandomDirection::random();

			Math::Vector3f translation = Math::RandomDirection::getVector(theta, phi)*light0.getRadius();

			return PointLight(light0.position()+translation,light0.color());
		}

		std::vector<PointLight> lightSampling() {
			std::vector< PointLight> lights;

			bool allLight = true;

			if (allLight) {
				for (LightSampler& sampler : m_lightSamplers) {
					lights.push_back(sampler.generate());
				}
				for (PointLight & light0 : m_lights) {
					lights.push_back(samplingSphere(light0));
				}
			}
			else {
				double random = Math::RandomDirection::random();
				double mem_score = 0.0;
				for (LightSampler& sampler : m_lightSamplers) {
					mem_score += sampler.getScore() / m_scoreLight;
					if (mem_score > random) {
						lights.push_back(sampler.generate());
						break;
					}
				}
				if (lights.empty()) {
					for (PointLight & light0 : m_lights) {
						mem_score += light0.getScore() / m_scoreLight;
						if (mem_score > random) {
							lights.push_back(samplingSphere(light0));
							break;
						}
					}
				}
			}

			return lights;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	RGBColor Scene::phong_direct(Ray const & ray, RayTriangleIntersection& intersection)
		///
		/// \brief	Renvoie la couleur issu du calcul d'ombrage de phong
		///
		/// \author	S. Corentin, L. Thomas
		/// \date	26/09/2018
		///
		/// \param	ray Le rayon lancé
		/// \param	intersection Le triangle intersecté avec le rayon
		/// \return	La couleur de phong
		////////////////////////////////////////////////////////////////////////////////////////////////////
		RGBColor phong_direct(Ray const & ray, RayTriangleIntersection& intersection, int depth) {

			//Initialisation des variables
			RGBColor result(0.0, 0.0, 0.0); //Couleur de retour

			const Triangle* tri = intersection.triangle();
			Material* material = tri->material(); //Materiau du triangle
			Math::Vector3f normal = tri->sampleNormal(intersection.uTriangleValue(), intersection.vTriangleValue(), ray.source()); //Normal au triangle
			Math::Vector3f intersect_point = intersection.intersection(); //Point d'intersection sur le triangle
			Math::Vector3f viewer = (ray.source() - intersect_point).normalized(); //Direction de la source

			std::vector<PointLight> lights = lightSampling();
			
			for (PointLight & light : lights) {

				//Valeur d'ombrage
				double shadow = ombrage(intersection,light);

				//Direction de la light
				double distance = (light.position() - intersect_point).norm();
				Math::Vector3f light_dir = (light.position() - intersect_point).normalized();
				if (normal*light_dir < 0)
				{
					continue;
				}

				double G = (normal*light_dir) / (distance + 1);

				RGBColor texture = intersection.triangle()->sampleTexture(intersection.uTriangleValue(), intersection.vTriangleValue());
				RGBColor brdf = material->BRDF(texture, ray.direction(), light_dir, normal);

				//light.computeScore();
				//double proba_light = light.getScore() / m_scoreLight;

				//Proba Light a revoir
				double proba_light = 2.f;

				result = result + light.color() * brdf * G * shadow / proba_light;				
			}

			result = result / double(lights.size());

			//Ajout de la couleur d'Emission
			result = result + material->getEmissive() / (intersection.tRayValue() + 1);

			return result;
		}



		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	RGBColor Scene::phong_indirect(Ray const & ray, RayTriangleIntersection& intersection,int depth,int maxDepth,int diffuseSamples,int specularSamples)
		///
		/// \brief	Renvoie la couleur issu du calcul d'ombrage de phong
		///
		/// \author	S. Corentin, L. Thomas
		/// \date	26/09/2018
		///
		/// \param	ray Le rayon lancé
		/// \param	intersection Le triangle intersecté avec le rayon
		///
		/// \return	La couleur de la composante indirect de l'illumination
		////////////////////////////////////////////////////////////////////////////////////////////////////
		RGBColor phong_indirect(Ray const & ray, RayTriangleIntersection const & intersection,int depth,int maxDepth,int diffuseSamples,int specularSamples) {
			
			RGBColor result;
			Material* material = intersection.triangle()->material();

			double rouletteRusse = Math::RandomDirection::random();
			double alpha = material->getAbsorption();
			if (rouletteRusse > alpha) {

				Math::Vector3f normal = intersection.triangle()->sampleNormal(intersection.uTriangleValue(), intersection.vTriangleValue(), ray.source()).normalized();
				Math::Vector3f reflected = Triangle::reflectionDirection(normal, ray.direction()).normalized();

				Math::Vector3f indirect = Math::RandomDirection(reflected, material->getShininess()).generate().normalized();

				Ray rayIndirect(intersection.intersection() + indirect * 0.001, indirect);

				RGBColor texture = intersection.triangle()->sampleTexture(intersection.uTriangleValue(), intersection.vTriangleValue());

				RGBColor brdf = material->BRDF(texture, ray.direction(), indirect, normal);

				float cos = normal * indirect;

				result = result + sendRay(rayIndirect, depth + 1, maxDepth, diffuseSamples, specularSamples)*brdf*cos;
			}

			result = result * (1 / (1 - alpha));

			return result;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	RGBColor Scene::sendRay(Ray const & ray, double limit, int depth, int maxDepth)
		///
		/// \brief	Sends a ray in the scene and returns the computed color
		///
		/// \author	F. Lamarche, S. Corentin, L. Thomas Université de Rennes 1
		/// \date	04/12/2013
		///
		/// \param	ray			The ray.
		/// \param	depth   	The current depth.
		/// \param	maxDepth	The maximum depth.
		///
		/// \return	The computed color.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		RGBColor sendRay(Ray const & ray, int depth, int maxDepth, int diffuseSamples, int specularSamples)
		{
			RGBColor result(0.0, 0.0, 0.0);
			RGBColor brouillard(0.1, 0.1,0.1);
			double di = 0.;
			double df = 30.;
			bool brouill = false;

			if(depth<=maxDepth){

				//Calcul de l'intersection
				RayTriangleIntersection intersection;
				if (accelerator.intersect(ray, intersection)) {

					//Calcul des rayon reflechis
					Math::Vector3f normal = intersection.triangle()->sampleNormal(intersection.uTriangleValue(), intersection.vTriangleValue(), ray.source());
					Math::Vector3f reflected = Triangle::reflectionDirection(normal, ray.direction());
					Ray reflexion(intersection.intersection()+reflected*0.001, reflected);

					Material* material = intersection.triangle()->material();

					result = phong_direct(ray, intersection,depth);

					if (!material->getSpecular().isBlack()) {
						result = result + material->getSpecular()*sendRay(reflexion, depth + 1, maxDepth, diffuseSamples, specularSamples);
					}

					result = result + phong_indirect(ray, intersection, depth + 1, maxDepth, diffuseSamples, specularSamples);
					/*
					if (brouill) {
						double d = intersection.tRayValue();
						double f = 0.;
						if (d < di) {
							f = 1.;
						}
						else if (d < df) {
							f = (df - d) / (df - di);
						}
						result = result * f + brouillard * (1 - f);
					}
					*/
				}
				/*
				else {
					if (brouill) {
						result = brouillard;
					}
					else {
						RGBColor background = RGBColor();
						if (skybox->isValid()) {
							int u = (ray.direction().normalized()[1]+1)*skybox->getSize()[0];
							int v = (ray.direction().normalized()[2] + 1)*skybox->getSize()[1];
							background = (skybox->pixel(u, v)/10)*0.5+brouillard*0.8;
						}
						result = background;
					}
					
				}*/
			}
			return result;
		}




		void save(::std::vector<::std::vector<::std::pair<int, RGBColor> > >& pixelTable, std::string nomFichier = "..\\..\\Resultat\\IlluminationGlobale\\RaytracingCPU.ppm")
		{
			std::ofstream fichier(nomFichier, std::ios::out | std::ios::trunc);  // on ouvre le fichier en lecture

			// Output FB as Image




			if (fichier)  // si l'ouverture a réussi
			{
				std::cout << "Debut de l'enregistrement" << std::endl;
				fichier << "P3\n" << m_visu->height() << " " << m_visu->width() << "\n255\n";
				for (int y = 0; y < m_visu->height(); y++) {
					for (int x = 0; x < m_visu->width(); x++) {
						//size_t pixel_index = y * m_visu->height() + x;
						//size_t pixel_index = 0;
						float r = 10 * pixelTable[x][y].second[0] / (double)(pixelTable[x][y].first);
						float g = 10 * pixelTable[x][y].second[1] / (double)(pixelTable[x][y].first);
						float b = 10 * pixelTable[x][y].second[2] / (double)(pixelTable[x][y].first);
						int ir = int(255 * (r / (r + 1)));
						int ig = int(255 * (g / (g + 1)));
						int ib = int(255 * (b / (b + 1)));
						fichier << ir << " " << ig << " " << ib << "\n";
					}
				}
				fichier.close();  // on ferme le fichier
			}
			else {  // sinon
				std::cerr << "Impossible d'ouvrir le fichier !" << std::endl;
			}

			std::cout << "Enregistrement termine" << std::endl;
		}


		double energie()
		{
			double result = 0.;
			for (int y = 0; y < m_visu->height(); y++)
			{
				for (int x = 0; x < m_visu->width(); x++)
				{
					if (x > 0 && x < m_visu->width()-1 && y > 0 && y < m_visu->height()-1)
					{
						result = result + abs((pixelTable[x][y].second / (double)(pixelTable[x][y].first) * 10).grey() - (pixelTable[x - 1][y].second / (double)(pixelTable[x - 1][y].first)*10).grey());
						result = result + abs((pixelTable[x][y].second / (double)(pixelTable[x][y].first) * 10).grey() - (pixelTable[x + 1][y].second / (double)(pixelTable[x + 1][y].first) * 10).grey());
						result = result + abs((pixelTable[x][y].second / (double)(pixelTable[x][y].first) * 10).grey() - (pixelTable[x][y - 1].second / (double)(pixelTable[x][y - 1].first) * 10).grey());
						result = result + abs((pixelTable[x][y].second / (double)(pixelTable[x][y].first) * 10).grey() - (pixelTable[x][y + 1].second / (double)(pixelTable[x][y + 1].first) * 10).grey());
					}
				}
			}
			//std::cout << result << std::endl;
			//std::cout << 100*abs(energieLastPass - result) / result << std::endl;
			return result;
		}

		void SaveEnergie(std::string energieString, std::string passCounter)
		{
			std::ofstream fichier(m_name+ std::to_string(m_pass) + ".ods", std::ios::out | std::ios::trunc);  // on ouvre le fichier en lecture

			// Output FB as Image




			if (fichier)  // si l'ouverture a réussi
			{	
				fichier << passCounter << ";\n" << energieString << ";\n";
				
				fichier.close();  // on ferme le fichier
			}
			else {  // sinon
				std::cerr << "Impossible d'ouvrir le fichier !" << std::endl;
			}

			std::cout << "Enregistrement termine" << std::endl;
		}


		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	void Scene::eventCompute()
		///
		/// \brief	gestion des events aux claviers pour déplacement dans la scène
		/// 		
		/// \author	S'tout mo vie
		/// \date	11/11/2018
		///
		////////////////////////////////////////////////////////////////////////////////////////////////////
		void eventCompute(std::vector<SDL_Event> events)
		{
			
			SDL_Event event;
			bool done = false;
			Math::Vector3f delta;

			//while (!done){
				for(SDL_Event event : events){
					switch (event.type) {
					case SDL_KEYDOWN:
						switch (event.key.keysym.sym) {

						case SDLK_s:
							save(pixelTable, m_name + std::to_string(m_pass)+".ppm");
							SaveEnergie(energieString, passCounter);
							done = true;
							break;
						}

					default:
						break;
					}
				}/*while*/
			//}
		}

		void loadTexture(std::string s) {
			if (skybox != nullptr) {
				delete skybox;
			}
			skybox = new Texture(s);
		}


		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	void Scene::compute(int maxDepth)
		///
		/// \brief	Computes a rendering of the current scene, viewed by the camera.
		/// 		
		/// \author	F. Lamarche, Université de Rennes 1
		/// \date	04/12/2013
		///
		/// \param	maxDepth	The maximum recursive depth.
		/// \param  subPixelDivision subpixel subdivisions to handle antialiasing
		////////////////////////////////////////////////////////////////////////////////////////////////////
		void compute(int maxDepth, int subPixelDivision = 1, int passPerPixel = 1)
		{
			
			//Chargement de la texture de Skybox
			Texture text("..\\..\\Models\\Skybox\\Skybox1.png");
			skybox=&text;
			//skybox = &Texture("");
			//loadTexture("..\\..\\Models\\Skybox\\Skybox1.png")

			for (::std::pair<BoundingBox, Geometry>& geometry : m_geometries) {
				LightSampler lightsampler;
				for (const Triangle & t : geometry.second.getTriangles()) {
					lightsampler.add(t);
				}
				if(lightsampler.hasLights())
					m_lightSamplers.push_back(lightsampler);
			}

			std::cout << m_lightSamplers.size() + m_lights.size() << " lights" << std::endl;

			for (LightSampler& sampler : m_lightSamplers) {
				sampler.computeScore();
				m_scoreLight += sampler.getScore();
			}
			for (PointLight & light0 : m_lights) {
				light0.computeScore();
				m_scoreLight += light0.getScore();
			}

			//Calcul de la structure d'optimisation
			accelerator.init(&m_geometries);

			// Step on x and y for subpixel sampling
			double step = 1.0f/subPixelDivision ;
			// Table accumulating values computed per pixel (enable rendering of each pass)
			pixelTable = ::std::vector<::std::vector<::std::pair<int, RGBColor> > >(m_visu->width(), ::std::vector<::std::pair<int, RGBColor> >(m_visu->width(), ::std::make_pair(0, RGBColor()))) ;

			// 1 - Rendering time
			LARGE_INTEGER frequency;        // ticks per second
			LARGE_INTEGER t1, t2;           // ticks
			double elapsedTime;
			// get ticks per second
			QueryPerformanceFrequency(&frequency);
			// start timer
			QueryPerformanceCounter(&t1);
			// Rendering pass number
			m_pass = 0;

			int nextCheck = 10;

			double energiePass = std::numeric_limits<double>::max()/(double)2;

			// Rendering
			//for(int passPerPixelCounter = 0 ; passPerPixelCounter<passPerPixel ; ++passPerPixelCounter)
			int passPerPixelCounter = 0;
			{
				//for (double xp = -0.5; xp < 0.5; xp += step)
				
				{
					//for (double yp = -0.5; yp < 0.5; yp += step)
					
					while((abs(energieLastPass-energiePass)/energiePass)> 0.01)
					{

						double xp = Math::RandomDirection::random() - 0.5;
						double yp = Math::RandomDirection::random() - 0.5;
						
						energieString = energieString + std::to_string(int(energie())) + ",";
						passCounter = passCounter + std::to_string(m_pass) + ",";

						if (m_pass == nextCheck)
						{
							energieLastPass = energiePass;
							energiePass = energie();
							nextCheck = int(nextCheck + 20);
						}
						//energie();
						::std::cout << "Pass: " << m_pass << "/" << passPerPixel * subPixelDivision * subPixelDivision << ::std::endl;
						++m_pass;
						// Sends primary rays for each pixel (uncomment the pragma to parallelize rendering)
#pragma omp parallel for schedule(dynamic)//, 10)//guided)//dynamic)
						for (int y = 0; y < m_visu->height(); y++)
						{
							for (int x = 0; x < m_visu->width(); x++)
							{
#pragma omp critical (visu)
								m_visu->plot(x, y, RGBColor(1000.0, 0.0, 0.0));
								// Ray casting
								RGBColor result = sendRay(m_camera.getRay(((double)x + xp) / m_visu->width(), ((double)y + yp) / m_visu->height()), 0, maxDepth, m_diffuseSamples, m_specularSamples);
								// Accumulation of ray casting result in the associated pixel
								::std::pair<int, RGBColor> & currentPixel = pixelTable[x][y];
								currentPixel.first++;
								currentPixel.second = currentPixel.second + result;
								// Pixel rendering (with simple tone mapping)
#pragma omp critical (visu)
								m_visu->plot(x, y, pixelTable[x][y].second / (double)(pixelTable[x][y].first) * 10);
								// Updates the rendering context (per pixel) - warning per pixel update can be costly...
//#pragma omp critical (visu)
								//m_visu->update();
							}
//#pragma omp critical (visu)
							//m_visu->update();
						}
						// Updates the rendering context (per pass)
						std::vector<SDL_Event> events = m_visu->update();
						eventCompute(events);
						
						
						// We print time for each pass
						QueryPerformanceCounter(&t2);
						elapsedTime = (double)(t2.QuadPart - t1.QuadPart) / (double)frequency.QuadPart;
						double remainingTime = (elapsedTime / m_pass)*(passPerPixel * subPixelDivision * subPixelDivision - m_pass);
						::std::cout << "time: " << elapsedTime << "s. " <<", remaining time: "<< remainingTime << "s. " <<", total time: "<< elapsedTime + remainingTime << ::std::endl;

						
					}
				}
			}
			// stop timer
			QueryPerformanceCounter(&t2);
			elapsedTime = (double)(t2.QuadPart - t1.QuadPart) / (double)frequency.QuadPart;
			::std::cout<<"time: "<<elapsedTime<<"s. "<<::std::endl ;


			save(pixelTable, m_name + std::to_string(m_pass) + ".ppm");

			SaveEnergie(energieString, passCounter);

		}




		


		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	void Scene::mouvement()
		///
		/// \brief	gestion des events aux claviers pour déplacement dans la scène
		/// 		
		/// \author	S'tout mo vie
		/// \date	11/11/2018
		///
		////////////////////////////////////////////////////////////////////////////////////////////////////
		void mouvement()
		{
			SDL_Event event;
			bool done = false;
			Math::Vector3f delta;
			
			Math::Vector3f target = m_camera.getTarget();
			Math::Vector3f position = m_camera.getPosition();
			double t0 = (double)target[0];
			double t1 = (double)target[1];

			double alpha = 0.1;
			double t = 0.1;
			double tmp;

			while (!done) {
				while (SDL_PollEvent(&event)) {
					switch (event.type) {
					case SDL_KEYDOWN:
						switch (event.key.keysym.sym) {

						case SDLK_UP:
							delta[0] = 0;
							delta[1] = vitesse;
							delta[2] = 0;
							m_camera.translateLocal(delta);
							break;

						case SDLK_RIGHT :
							delta[0] = vitesse;
							delta[1] = 0.;
							delta[2] = 0;
							m_camera.translateLocal(delta);
							break;
							
						case SDLK_DOWN:
							delta[0] = 0;
							delta[1] = - vitesse;
							delta[2] = 0;
							m_camera.translateLocal(delta);
							break;

						case SDLK_LEFT:
							delta[0] = - vitesse;
							delta[1] = 0.;
							delta[2] = 0;
							m_camera.translateLocal(delta);
							break;

						case SDLK_q:
							target[0] = position[0] + ((t0 - position[0])*cos(alpha) - (t1 - position[1])*sin(alpha));
							target[1] = position[1] + ((t0 - position[0]) * sin(alpha) + (t1 - position[1]) * cos(alpha));
							m_camera.setTarget(target);
							break;

						case SDLK_d:
							alpha *= -1;
							target[0] = position[0] + ((t0 - position[0])*cos(alpha) - (t1 - position[1])*sin(alpha));
							target[1] = position[1] + ((t0 - position[0]) * sin(alpha) + (t1 - position[1]) * cos(alpha));
							m_camera.setTarget(target);
							break;

						case SDLK_z:
							tmp = ((target[2] - position[2]) / sqrt(pow((target[1] - position[1]),2) + pow((target[0] - position[0]), 2)));
							//while (tmp > Math::pi/2.) tmp -= 2 * Math::pi;
							//while (tmp < Math::pi / 2.) tmp += 2 * Math::pi;
							alpha = atan(tmp);
							if (alpha + t < 3.14 / 2.)
							{
								delta[0] = target[0];
								delta[1] = target[1];
								delta[2] = tan(alpha + t)*sqrt(pow((target[1] - position[1]), 2) + pow((target[0] - position[0]), 2))+ position[2];
								//std::cout << tmp << std::endl;
								//std::cout << alpha << std::endl;
								//std::cout << alpha + t << std::endl;
								m_camera.setTarget(delta);
							}
							break;

						case SDLK_s:
							t *= -1;
							alpha = atan((target[2] - position[2]) / sqrt(sqrt(pow((target[1] - position[1]), 2) + pow((target[0] - position[0]), 2))));
							if (alpha + t > -3.14 / 2.)
							{
								delta[0] = target[0];
								delta[1] = target[1];
								delta[2] = tan(alpha + t)*sqrt(pow((target[1] - position[1]), 2) + pow((target[0] - position[0]), 2)) + position[2];
								m_camera.setTarget(delta);
							}
							break;

						case SDLK_LSHIFT:
							vitesse = vitesse * 1.5;
							std::cout << vitesse << std::endl;
							break;

						case SDLK_LCTRL:
							vitesse = vitesse * 0.66;
							std::cout << vitesse << std::endl;
							break;
						}

					case SDL_QUIT:
						done = true;
						break;

						break;
					default:
						break;
					}

					std::cout << "position : " << position[0] << " " << position[1] << " " << position[2] << std::endl;
					std::cout << "position : " << target[0] << " " << target[1] << " " << target[2] << std::endl;
				}/*while*/
			}/*while(!done)*/
		}


		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	void Scene::mouvement()
		///
		/// \brief	gestion des events aux claviers pour déplacement dans la scène
		/// 		
		/// \author	S'tout mo vie
		/// \date	11/11/2018
		///
		////////////////////////////////////////////////////////////////////////////////////////////////////
		void computeTempsReel(int maxDepth,double factor=1.)
		{
			Texture text("..\\..\\Models\\Skybox\\Skybox1.png");
			skybox = &text;

			for (::std::pair<BoundingBox, Geometry>& geometry : m_geometries) {
				LightSampler lightsampler;
				for (const Triangle & t : geometry.second.getTriangles()) {
					lightsampler.add(t);
				}
				if (lightsampler.hasLights())
					m_lightSamplers.push_back(lightsampler);
			}

			std::cout << m_lightSamplers.size() + m_lights.size() << " lights" << std::endl;

			for (LightSampler& sampler : m_lightSamplers) {
				sampler.computeScore();
				m_scoreLight += sampler.getScore();
			}
			for (PointLight & light0 : m_lights) {
				light0.computeScore();
				m_scoreLight += light0.getScore();
			}
			//Calcul de la structure d'optimisation
			accelerator.init(&m_geometries);

			int h = (int)(m_visu->width() / factor);
			int w = (int)(m_visu->height() / factor);

			::std::vector<::std::vector<RGBColor>>pixelTable (h, ::std::vector<RGBColor>(w));

			SDL_Event event;
			bool done = false;

			while (!done) {

				// Calcul des pixels
#pragma omp parallel for schedule(dynamic)//, 10)//guided)//dynamic)
				for (int y = 0; y < w; y++)
				{
					for (int x = 0; x < h; x++)
					{
						RGBColor result = sendRay(m_camera.getRay(((double)x) /h, ((double)y) / w), 0, maxDepth, m_diffuseSamples, m_specularSamples);
						pixelTable[x][y] = result*10;
					}
				}

#pragma omp parallel for schedule(dynamic)
				for (int y = 0; y < m_visu->height(); y++)
				{
					for (int x = 0; x < m_visu->width(); x++)
					{

						//Interpolation bilinéaire
						double Rx = (double) x / factor;
						double Ry = (double)y / factor;

						int x_min = (int) Rx;
						int x_max = (int)Rx + 1;
						if (x_max >= h) x_max--;
						int y_min = (int)Ry;
						int y_max = (int)Ry + 1;
						if (y_max >= w) y_max--;

						RGBColor p1 = pixelTable[x_min][y_min];
						RGBColor p2 = pixelTable[x_max][y_min];
						RGBColor p3 = pixelTable[x_min][y_max];
						RGBColor p4 = pixelTable[x_max][y_max];

						double dX = Rx - x_min;
						RGBColor p1_2 = p1 * (1 - dX) + p2 * (dX);

						RGBColor p3_4 = p3 * (1 - dX) + p4 * (dX);

						double dY = Ry - y_min;

						RGBColor result = p1_2 * (1 - dY) + p3_4 * (dY);


#pragma omp critical (visu)
						m_visu->plot(x,y, result);

					}
				}

				// Updates the rendering context (per pass)
				m_visu->update();

				mouvement();

				if (SDL_PollEvent(&event)) {
					if (event.type == SDL_QUIT) {
						done = true;
					}

				}
			}
		}

		void computeMovie(int maxDepth, int subPixelDivision = 1, int passPerPixel = 1)
		{
			//Chargement de la texture de Skybox
			Texture text("..\\..\\Models\\Skybox\\Skybox1.png");
			skybox = &text;
			//skybox = &Texture("");
			//loadTexture("..\\..\\Models\\Skybox\\Skybox1.png")

			for (::std::pair<BoundingBox, Geometry>& geometry : m_geometries) {
				LightSampler lightsampler;
				for (const Triangle & t : geometry.second.getTriangles()) {
					lightsampler.add(t);
				}
				if (lightsampler.hasLights())
					m_lightSamplers.push_back(lightsampler);
			}

			std::cout << m_lightSamplers.size() + m_lights.size() << " lights" << std::endl;

			for (LightSampler& sampler : m_lightSamplers) {
				sampler.computeScore();
				m_scoreLight += sampler.getScore();
			}
			for (PointLight & light0 : m_lights) {
				light0.computeScore();
				m_scoreLight += light0.getScore();
			}

			//Calcul de la structure d'optimisation
			accelerator.init(&m_geometries);

			// Step on x and y for subpixel sampling
			double step = 1.0f / subPixelDivision;
			// Table accumulating values computed per pixel (enable rendering of each pass)
			pixelTable = ::std::vector<::std::vector<::std::pair<int, RGBColor> > >(m_visu->width(), ::std::vector<::std::pair<int, RGBColor> >(m_visu->width(), ::std::make_pair(0, RGBColor())));

			// 1 - Rendering time
			LARGE_INTEGER frequency;        // ticks per second
			LARGE_INTEGER t1, t2;           // ticks
			double elapsedTime;
			// get ticks per second
			QueryPerformanceFrequency(&frequency);
			// start timer
			QueryPerformanceCounter(&t1);
			// Rendering pass number
			m_pass = 0;

			int nextCheck = 10;

			double energiePass = std::numeric_limits<double>::max() / (double)2;


			std::vector< Math::Vector3f> positions;
			positions.push_back(Math::makeVector(-4.f, 0.f, 0.f));
			positions.push_back(Math::makeVector(-3.97f, 0.16f, 0.f));
			positions.push_back(Math::makeVector(-3.95f, 0.21f, 0.f));
			positions.push_back(Math::makeVector(-3.93f, 0.32f, 0.f));
			positions.push_back(Math::makeVector(-3.89f, 0.48f, 0.f));

			std::vector< Math::Vector3f> targets;
			targets.push_back(Math::makeVector(-3., 0., 0.));
			targets.push_back(Math::makeVector(-2.97, 0.06, 0.));
			targets.push_back(Math::makeVector(-2.99, -0.04, 0.));
			targets.push_back(Math::makeVector(-2.98, 0.03, 0.));
			targets.push_back(Math::makeVector(-2.96, 0.09, 0.));


			// Rendering
			//for(int passPerPixelCounter = 0 ; passPerPixelCounter<passPerPixel ; ++passPerPixelCounter)
			int passPerPixelCounter = 0;
			{
				//for (double xp = -0.5; xp < 0.5; xp += step)
				for (int p = 0; p < positions.size(); ++p)
				{
					m_camera.setPosition(positions[p]);
					m_camera.setTarget(targets[p]);


					//for (double yp = -0.5; yp < 0.5; yp += step)

					while ((abs(energieLastPass - energiePass) / energiePass) > 0.2)
					{
						double xp = Math::RandomDirection::random() - 0.5;
						double yp = Math::RandomDirection::random() - 0.5;
						if (m_pass == nextCheck)
						{
							energieLastPass = energiePass;
							energiePass = energie();
							nextCheck = int(nextCheck + 20);
						}
						//energie();
						::std::cout << "Pass: " << m_pass << "/" << passPerPixel * subPixelDivision * subPixelDivision << ::std::endl;
						++m_pass;
						// Sends primary rays for each pixel (uncomment the pragma to parallelize rendering)
#pragma omp parallel for schedule(dynamic)//, 10)//guided)//dynamic)
						for (int y = 0; y < m_visu->height(); y++)
						{
							for (int x = 0; x < m_visu->width(); x++)
							{
#pragma omp critical (visu)
								m_visu->plot(x, y, RGBColor(1000.0, 0.0, 0.0));
								// Ray casting
								RGBColor result = sendRay(m_camera.getRay(((double)x + xp) / m_visu->width(), ((double)y + yp) / m_visu->height()), 0, maxDepth, m_diffuseSamples, m_specularSamples);
								// Accumulation of ray casting result in the associated pixel
								::std::pair<int, RGBColor> & currentPixel = pixelTable[x][y];
								currentPixel.first++;
								currentPixel.second = currentPixel.second + result;
								// Pixel rendering (with simple tone mapping)
#pragma omp critical (visu)
								m_visu->plot(x, y, pixelTable[x][y].second / (double)(pixelTable[x][y].first) * 10);
								// Updates the rendering context (per pixel) - warning per pixel update can be costly...
//#pragma omp critical (visu)
								//m_visu->update();
							}
							//#pragma omp critical (visu)
														//m_visu->update();
						}
						// Updates the rendering context (per pass)
						std::vector<SDL_Event> events = m_visu->update();
						eventCompute(events);


						// We print time for each pass
						QueryPerformanceCounter(&t2);
						elapsedTime = (double)(t2.QuadPart - t1.QuadPart) / (double)frequency.QuadPart;
						double remainingTime = (elapsedTime / m_pass)*(passPerPixel * subPixelDivision * subPixelDivision - m_pass);
						::std::cout << "time: " << elapsedTime << "s. " << ", remaining time: " << remainingTime << "s. " << ", total time: " << elapsedTime + remainingTime << ::std::endl;


					}
					save(pixelTable, m_name + "Movie\\" + std::to_string(p) + ".ppm");
					energieLastPass = std::numeric_limits<double>::max();
					pixelTable = ::std::vector<::std::vector<::std::pair<int, RGBColor> > >(m_visu->width(), ::std::vector<::std::pair<int, RGBColor> >(m_visu->width(), ::std::make_pair(0, RGBColor())));
				}



			}
			// stop timer
			QueryPerformanceCounter(&t2);
			elapsedTime = (double)(t2.QuadPart - t1.QuadPart) / (double)frequency.QuadPart;
			::std::cout << "time: " << elapsedTime << "s. " << ::std::endl;


			save(pixelTable, m_name + "Movie\\" + std::to_string(m_pass) + ".ppm");

		}


	} ;
}

#endif
