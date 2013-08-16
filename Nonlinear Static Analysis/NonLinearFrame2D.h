#pragma once

#ifndef NonLinearFrame2D_H
#define NonLinearFrame2D_H

#include "ElasticFrame2d.h"

class NonLinearFrame2D : public ElasticFrame2D
{
	protected:
		vector <Node2D> MyVectorInitialNodes;

		vector <NodeLoad2D> MyVectorTotalNodeLoads;

		vector <MomentLoad2D> MyVectorTotalMomentLoads;
		vector <PointLoad2D> MyVectorTotalPointLoads;
		vector <PyramidLoad2D> MyVectorTotalPyramidLoads;
		vector <TriangularLoad2D> MyVectorTotalTriangularLoads;
		vector <UniformLoad2D> MyVectorTotalUniformLoads;

	public:
		NonLinearFrame2D(void);
		~NonLinearFrame2D(void);

		NonLinearFrame2D& operator= (const NonLinearFrame2D &Frame);

		void operator+= (const ElasticFrame2D &Frame);
};

#endif
