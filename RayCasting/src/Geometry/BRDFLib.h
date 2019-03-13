
#include <Geometry/RGBColor.h>
#include <Geometry/Texture.h>
#include <Math/Vector.h>
#include <Geometry/Material.h>
#include <Geometry/RayTriangleIntersection.h>
#include <Geometry/Ray.h>

namespace Geometry
{
	class BRDFLib {
	public :
		BRDFLib() {

		}

		static RGBColor computeColor(Ray const & ray, RayTriangleIntersection const & intersection,Math::Vector3f sortie) {
			RGBColor result =RGBColor(0.0,0.0,0.0);

			Material* material = intersection.triangle()->material();
			RGBColor texture = intersection.triangle()->sampleTexture(intersection.uTriangleValue(), intersection.vTriangleValue());
			Math::Vector3f normal = intersection.triangle()->sampleNormal(intersection.uTriangleValue(), intersection.vTriangleValue(), ray.source()).normalized();

			Math::Vector3f entree = ray.direction();
			Math::Vector3f reflechit = Triangle::reflectionDirection(normal, ray.direction()).normalized();
			float theta = sortie * reflechit;
			Math::Vector3f h = (entree + sortie).normalized();
			float fresnel = material->getFresnel() + (1 - material->getFresnel())*pow(1 - (entree*h), 5);
			float D = 1 / pow( material->getTaille()* cos(material->getRepartition()) , 2 ) * std::exp(-1*pow((std::tan(material->getRepartition())),2));
			float G = std::min(std::min(2*(normal*h)*(normal*entree)/ (h*entree), 2 * (normal*h)*(normal*sortie) / (h*entree)),1.);

			int id= material->getId();
			switch (id) {
				//Phong
				case 0 :
					result = material->getDiffuse() * (normal * sortie) + material->getSpecular() * pow(abs(theta), material->getShininess());
					result = result * texture;
					break;
				//100% Diffus
				case 1 :
					result = material->getDiffuse()*texture* (normal * sortie);
					break;
				case 2 :
					result = material->getDiffuse()*texture;
					result = result * (pow(abs(theta), 3) - pow(0.9*abs(theta), 5)) / 0.37 * (normal * sortie);
					break;
				//Fresnel
				case 3 :
					//Fresnel
					result = material->getDiffuse()*texture*fresnel;
					break;
				case 4:
					//Cook
					result = material->getDiffuse()*texture*abs(fresnel*D*G/(Math::pi*(normal*sortie)*(normal*entree)));
					break;
				default: 
					break;
			}
			return result;
		}

		static Math::Vector3f reflectionDirection(Math::Vector3f const & n, Math::Vector3f const & dir)
		{
			Math::Vector3f reflected(dir - n * (2.0f*(dir*n)));
			return reflected;
		}

	};
}