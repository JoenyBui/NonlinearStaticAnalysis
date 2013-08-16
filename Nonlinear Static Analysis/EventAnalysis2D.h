#pragma once

#ifndef EventAnalysis2D_H
#define EventAnalysis2D_H

#include "stdafx.h"
#include "ElasticFrame2D.h"

class EventAnalysis2D
{
	private:

	protected:
		// An array of the Plastic Frame Analysis.
		vector <ElasticFrame2D> MyPlasticFrame;
		
		// A vector that describes the Static Load.  Loads are applied at time step 0.
		vector <NodeLoad2D> MyStaticNodeLoads;
		vector <NodeLoad2D> MyDynamicNodeLoads;

		// Pointers of the Dynamic Beam Loading.
		vector <BeamLoad2D*> MyStaticBeamLoads;
		vector <BeamLoad2D*> MyDynamicBeamLoads;

		// Storing the Static Beam Loading.
		vector <MomentLoad2D> MyStaticMomentLoads;
		vector <PointLoad2D> MyStaticPointLoads;
		vector <PyramidLoad2D> MyStaticPyramidLoads;
		vector <TriangularLoad2D> MyStaticTriangularLoads;
		vector <UniformLoad2D> MyStaticUniformLoads;

		// Storing the Dynamic Loading.
		vector <MomentLoad2D> MyDynamicMomentLoads;
		vector <PointLoad2D> MyDynamicPointLoads;
		vector <PyramidLoad2D> MyDynamicPyramidLoads;
		vector <TriangularLoad2D> MyDynamicTriangularLoads;
		vector <UniformLoad2D> MyDynamicUniformLoads;

		// Store as Incremental Values as Values, not as Pointers.
		// 1 - Is the Time Step.
		// 2 - Is the Nodal Array.
		vector <vector <NodeLoad2D>> MyIncrementalNodeLoads;
		vector <vector <BeamLoad2D*>> MyIncrementalBeamLoads;

		// Time Step and Duration.
		vector <double> MyTimeDuration;

		// Initialize the Vector Beam Local Forces
		vector <mat> MyVectorBeamLocalForces;
	public:
		void Initialize();

		void Clone(const EventAnalysis2D &Event);

		EventAnalysis2D(void);

		~EventAnalysis2D(void);
		
		void InitializeBeamForceVector();

		void SetTimeDuration(vector <double> TimeDuration);

		void SetTimeDuration(int NumberOfTimeStep, int TimeDuratin);

		// Set the Static Loads.
		void SetStaticLoads();
		
		// Get the Incremental Loads for the Time Step.
		void SetIncrementNodeLoads(int TimeStep);
		void SetIncrementBeamLoads(int TimeStep);

		// Set the Geometry of the frame of the structure.
		void SetFrameGeometry(ElasticFrame2D Frame);
		
		// Add the Static Load at '0' time.
		void AddStaticLoad(NodeLoad2D Load);
		void AddStaticLoad(MomentLoad2D Load);
		void AddStaticLoad(PointLoad2D Load);
		void AddStaticLoad(PyramidLoad2D Load);
		void AddStaticLoad(TriangularLoad2D Load);
		void AddStaticLoad(UniformLoad2D Load);

		void AddStaticLoad(vector <NodeLoad2D> Load);
		void AddStaticLoad(vector <MomentLoad2D> Load);
		void AddStaticLoad(vector <PointLoad2D> Load);
		void AddStaticLoad(vector <PyramidLoad2D> Load);
		void AddStaticLoad(vector <TriangularLoad2D> Load);
		void AddStaticLoad(vector <UniformLoad2D> Load);

		// Increment the Load.
		void AddIncrementalLoad(NodeLoad2D Load);
		void AddIncrementalLoad(MomentLoad2D Load);
		void AddIncrementalLoad(PointLoad2D Load);
		void AddIncrementalLoad(PyramidLoad2D Load);
		void AddIncrementalLoad(TriangularLoad2D Load);
		void AddIncrementalLoad(UniformLoad2D Load);

		void AddIncrementalLoad(vector <NodeLoad2D> Load);
		void AddIncrementalLoad(vector <MomentLoad2D> Load);
		void AddIncrementalLoad(vector <PointLoad2D> Load);
		void AddIncrementalLoad(vector <PyramidLoad2D> Load);
		void AddIncrementalLoad(vector <TriangularLoad2D> Load);
		void AddIncrementalLoad(vector <UniformLoad2D> Load);

		// Run the Dynamic Analysis.
		void RunAnalysis();

		
};

#endif
