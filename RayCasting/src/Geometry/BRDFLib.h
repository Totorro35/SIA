
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

			int id= material->getId();
			switch (id) {
				//Phong
				case 0 :
					result = material->getDiffuse() * (normal * sortie) + material->getSpecular() * pow(theta, material->getShininess());
					result = result * texture;
					break;
				//100% Diffus
				case 1 :
					result = material->getDiffuse()*texture* (normal * sortie);
					break;
				case 2 :
					result = material->getDiffuse()*texture;
					if (theta < 0) {
						//std::cout << "Au scandale!!" << std::endl;
					}
					result = result * (pow(abs(theta), 3) - pow(0.9*abs(theta), 5)) / 0.37 * (normal * sortie);
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