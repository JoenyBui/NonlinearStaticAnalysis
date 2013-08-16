#include "StdAfx.h"
#include "EventAnalysis2D.h"

///<summary>Run the Analysis.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::RunAnalysis()
{
	// Initialize the Beam Force Vector that holds the stresses on the beam.
	this -> InitializeBeamForceVector();

	// Set the Static Loads.
	this -> SetStaticLoads();

	// Nodal Displacement.
	vector <mat> NodalDisplacement;

	// Counter of the Hinge.
	bool Hinging = false;

	for (size_t i = 0; i < MyTimeDuration.size(); i++)
	{
		if (i != 0)
		{
			// Define the Geometry based off the previous configuration.
			MyPlasticFrame.push_back(MyPlasticFrame[i - 1].GetPlasticFrameGeometry());	
			
			// Check the Moment Capacity and Release Degrees of Freedom as needed.
			//MyPlasticFrame[i].CheckMomentCapacity(MyVectorBeamLocalForces, Hinging);
			
			// Set the ID of the frame.
			MyPlasticFrame[i].SetID(i);
		};

		// Set the Increment Load.
		this -> SetIncrementNodeLoads(i);

		// Solve for the displacement from the stiffness and the load vector.
		vector <mat> StaticResults = MyPlasticFrame[i].RunStaticAnalysis();
		
		StaticResults[8].print("Displacement:");
		StaticResults[9].print("Reaction:");

		// Retreive the load forces at each beam.
		vector <mat> BeamEndForces = MyPlasticFrame[i].GetInternalBeamEndForces();

		// Loop and add the Increment.
		for (size_t j = 0; j < BeamEndForces.size(); j++)
		{
			// Store the Local Forces.
			MyVectorBeamLocalForces[j] += BeamEndForces[j];
		};
		
		NodalDisplacement.push_back(StaticResults[8]);
	};

	for (size_t k = 0; k < NodalDisplacement.size(); k++)
	{
		NodalDisplacement[k].print("Displacement"); 
	};
};

/// <summary></summary>
void EventAnalysis2D::Initialize()
{
};

/// <summary></summary>
void EventAnalysis2D::Clone(const EventAnalysis2D &Event)
{
	
};

/// <summary></summary>
EventAnalysis2D::EventAnalysis2D(void)
{
};

/// <summary></summary>
EventAnalysis2D::~EventAnalysis2D(void)
{
};

/// <summary></summary>
void EventAnalysis2D::SetTimeDuration(std::vector<double> TimeDuration)
{
	// Set the Time Duration List.
	MyTimeDuration = TimeDuration;
};

/// <summary></summary>
void EventAnalysis2D::SetTimeDuration(int NumberOfTimeStep, int TimeDuration)
{
	// Loop through the Time Duration List.
	for (int i = 0; i < NumberOfTimeStep; i++)
	{
		// Provide a constant Time Duration.
		MyTimeDuration.push_back(TimeDuration);
	};
};

///<summary>Initialize the beam end force vector to zeros.</summary>
///<remarks>description</remarks>
void EventAnalysis2D::InitializeBeamForceVector()
{
	// Initialize Vector
	for (size_t i = 0; i < MyPlasticFrame[0].GetNumberOfBeams(); i++)
	{
		// Declare Beam Local Force Array.
		mat BeamLocalForces(6, 1);

		// Initialize Local Force Array.
		BeamLocalForces.fill(0.0);

		// Add to the Vector.
		MyVectorBeamLocalForces.push_back(BeamLocalForces);
	};
};



///<summary>Set the Static Load for the undeform shape of the frame.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::SetStaticLoads()
{
	MyPlasticFrame[0].AddNodeLoad(MyStaticNodeLoads);
	MyPlasticFrame[0].AddBeamLoad(MyStaticMomentLoads);
	MyPlasticFrame[0].AddBeamLoad(MyStaticPointLoads);
	MyPlasticFrame[0].AddBeamLoad(MyStaticPyramidLoads);
	MyPlasticFrame[0].AddBeamLoad(MyStaticTriangularLoads);
	MyPlasticFrame[0].AddBeamLoad(MyStaticUniformLoads);
};

/// <summary>Set the Incremental Loads.</summary>
void EventAnalysis2D::SetIncrementNodeLoads(int TimeStep)
{
	if (TimeStep != 0)
	{
		// Decalare a vector of Nodal Loads.
		vector <NodeLoad2D> NodeLoads;

		// Set the Incremental Nodal Load for the specific time step.
		for (size_t i = 0; i < MyDynamicNodeLoads.size(); i++)
		{
			// Return the Nodal Load Increment for the corresponding Nodal Load.
			NodeLoads.push_back(MyDynamicNodeLoads[i]);
		};

		// Store the Vector.
		MyIncrementalNodeLoads.push_back(NodeLoads);

		// Add the Nodal Loads.
		MyPlasticFrame[TimeStep].AddNodeLoad(NodeLoads);
	}
};

/// <summary>Store the geometry of the plastic frame in the 0 time step of the analysis.</summary>
void EventAnalysis2D::SetFrameGeometry(ElasticFrame2D Frame)
{
	MyPlasticFrame.push_back(Frame);
};

/// <summary>Add the Static Node Load.</summary>
void EventAnalysis2D::AddStaticLoad(NodeLoad2D Load)
{
	MyStaticNodeLoads.push_back(Load);
};

/// <summary>Add the Static Beam Load.</summary>
void EventAnalysis2D::AddStaticLoad(MomentLoad2D Load)
{
	MyStaticMomentLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddStaticLoad(PointLoad2D Load)
{
	MyStaticPointLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddStaticLoad(PyramidLoad2D Load)
{
	MyStaticPyramidLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddStaticLoad(TriangularLoad2D Load)
{
	MyStaticTriangularLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddStaticLoad(UniformLoad2D Load)
{
	MyStaticUniformLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddIncrementalLoad(NodeLoad2D Load)
{
	MyDynamicNodeLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddIncrementalLoad(MomentLoad2D Load)
{
	MyDynamicMomentLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddIncrementalLoad(PointLoad2D Load)
{
	MyDynamicPointLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddIncrementalLoad(PyramidLoad2D Load)
{
	MyDynamicPyramidLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddIncrementalLoad(TriangularLoad2D Load)
{
	MyDynamicTriangularLoads.push_back(Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void EventAnalysis2D::AddIncrementalLoad(UniformLoad2D Load)
{
	MyDynamicUniformLoads.push_back(Load);
};


