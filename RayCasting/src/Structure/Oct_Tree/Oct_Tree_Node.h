#include <Geometry/Geometry.h>
#include <Geometry/BoundingBox.h>
#include <queue>

	class Oct_Tree_Node
	{
	private:
		Oct_Tree_Node* m_branch[8];

	public:
		Oct_Tree_Node();
		void init(::std::deque<::std::pair<Geometry::BoundingBox, Geometry::Geometry>>& geometries);
		virtual ~Oct_Tree_Node();
	};
	