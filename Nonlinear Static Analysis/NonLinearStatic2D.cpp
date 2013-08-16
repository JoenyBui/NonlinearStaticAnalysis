#include "StdAfx.h"
#include "NonLinearStatic2D.h"

#pragma region "Constructors and Destructors"

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::Initialize()
{
	ShowInternalEndForce = false;
	ShowExternalEndForce = false;
};

///<summary></summary>
///<param name="Event"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::Clone(const NonLinearStatic2D &Event)
{
	// Set the Frame Geometry.
	this -> SetFrameGeometry(Event.MyBaseFrame);

	// Add Static Loads.
	this -> AddStaticLoad(Event.MyStaticNodeLoads);
	this -> AddStaticLoad(Event.MyStaticMomentLoads);
	this -> AddStaticLoad(Event.MyStaticPointLoads);
	this -> AddStaticLoad(Event.MyStaticPyramidLoads);
	this -> AddStaticLoad(Event.MyStaticTriangularLoads);
	this -> AddStaticLoad(Event.MyStaticUniformLoads);

	// Add Incremental Loads.
	this -> AddIncrementalLoad(Event.MyIncrementNodeLoads);
	this -> AddIncrementalLoad(Event.MyIncrementMomentLoads);
	this -> AddIncrementalLoad(Event.MyIncrementPointLoads);
	this -> AddIncrementalLoad(Event.MyIncrementPyramidLoads);
	this -> AddIncrementalLoad(Event.MyIncrementTriangularLoads);
	this -> AddIncrementalLoad(Event.MyIncrementUniformLoads);

	// *** POSITIVE ***
	MyVectorTotalDisplacementPositive = Event.MyVectorTotalDisplacementPositive;
	MyVectorTotalExternalForcesPositive = Event.MyVectorTotalExternalForcesPositive;
	MyVectorTotalInternalForcesPositive = Event.MyVectorTotalInternalForcesPositive;
	MyHingingHistoryPositive = Event.MyHingingHistoryPositive;

	MyLocationVectorPositiveTranslationX = Event.MyLocationVectorPositiveTranslationX;
	MyLocationVectorPositiveTranslationY = Event.MyLocationVectorPositiveTranslationY;
	MyLocationVectorPositiveRotationZ = Event.MyLocationVectorPositiveRotationZ;

	MyDisplacementVectorPositiveTranslationX = Event.MyDisplacementVectorPositiveTranslationX;
	MyDisplacementVectorPositiveTranslationY = Event.MyDisplacementVectorPositiveTranslationY;
	MyDisplacementVectorPositiveRotationZ = Event.MyDisplacementVectorPositiveRotationZ;

	MyShapeVectorPositiveTranslationX = Event.MyShapeVectorPositiveTranslationX;
	MyShapeVectorPositiveTranslationY = Event.MyShapeVectorPositiveTranslationY;
	MyShapeVectorPositiveRotationZ = Event.MyShapeVectorPositiveRotationZ;

	MyMassVectorPositiveTranslationX = Event.MyMassVectorPositiveTranslationX;
	MyMassVectorPositiveTranslationY = Event.MyMassVectorPositiveTranslationY;
	MyMassVectorPositiveRotationZ = Event.MyMassVectorPositiveRotationZ;

	MyBeamLoadVectorPositiveTranslationX = Event.MyBeamLoadVectorPositiveTranslationX;
	MyBeamLoadVectorPositiveTranslationY = Event.MyBeamLoadVectorPositiveTranslationY;
	MyBeamLoadVectorPositiveRotationZ = Event.MyBeamLoadVectorPositiveRotationZ;

	MyNodeLoadVectorPositiveTranslationX = Event.MyNodeLoadVectorPositiveTranslationX;
	MyNodeLoadVectorPositiveTranslationY = Event.MyNodeLoadVectorPositiveTranslationY;
	MyNodeLoadVectorPositiveRotationZ = Event.MyNodeLoadVectorPositiveRotationZ;

	// *** NEGATIVE ***
	MyHingingHistoryPositive = Event.MyHingingHistoryPositive;

	MyLocationVectorNegativeTranslationX = Event.MyLocationVectorNegativeTranslationX;
	MyLocationVectorNegativeTranslationY = Event.MyLocationVectorNegativeTranslationY;
	MyLocationVectorNegativeRotationZ = Event.MyLocationVectorNegativeRotationZ;

	MyDisplacementVectorNegativeTranslationX = Event.MyDisplacementVectorNegativeTranslationX;
	MyDisplacementVectorNegativeTranslationY = Event.MyDisplacementVectorNegativeTranslationY;
	MyDisplacementVectorNegativeRotationZ = Event.MyDisplacementVectorNegativeRotationZ;

	MyShapeVectorNegativeTranslationX = Event.MyShapeVectorNegativeTranslationX;
	MyShapeVectorNegativeTranslationY = Event.MyShapeVectorNegativeTranslationY;
	MyShapeVectorNegativeRotationZ = Event.MyShapeVectorNegativeRotationZ;

	MyMassVectorNegativeTranslationX = Event.MyMassVectorNegativeTranslationX;
	MyMassVectorNegativeTranslationY = Event.MyMassVectorNegativeTranslationY;
	MyMassVectorNegativeRotationZ = Event.MyMassVectorNegativeRotationZ;

	MyBeamLoadVectorNegativeTranslationX = Event.MyBeamLoadVectorNegativeTranslationX;
	MyBeamLoadVectorNegativeTranslationY = Event.MyBeamLoadVectorNegativeTranslationY;
	MyBeamLoadVectorNegativeRotationZ = Event.MyBeamLoadVectorNegativeRotationZ;

	MyNodeLoadVectorPositiveTranslationX = Event.MyNodeLoadVectorPositiveTranslationX;
	MyNodeLoadVectorPositiveTranslationY = Event.MyNodeLoadVectorPositiveTranslationY;
	MyNodeLoadVectorPositiveRotationZ = Event.MyNodeLoadVectorPositiveRotationZ;
};


///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
NonLinearStatic2D::NonLinearStatic2D(void)
{
	this -> Initialize();
}

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
NonLinearStatic2D::~NonLinearStatic2D(void)
{
}

///<summary>Copy the Constructor</summary>
///<param name="name"></param>
///<remarks>description</remarks>
NonLinearStatic2D::NonLinearStatic2D(const NonLinearStatic2D &Event)
{
	this -> Clone(Event);
};

///<summary>Store the Event.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
NonLinearStatic2D& NonLinearStatic2D::operator =(const NonLinearStatic2D &Event)
{
	// Call the Clone Subroutine.
	this -> Clone(Event);

	// Return a pointer to this Instance.
	return *this;
};
#pragma endregion

#pragma region "Analysis Methods"

///<summary>Run the Non-Linear Incremental Static Analysis for the Frame stepping through the Incremental load
/// using the Arc-Length Method ensuring equilibrium at each time step.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::RunAnalysis(int TypeOfNonLinear, 
									int NumOfIncrement, 
									double NormTolerance,
									int ExitCounterMax,
									double InitialIncrementLoadRatio)
{
	// Run the Static Analysis to get the Initial Deformation.
	this -> RunStaticAnalysis();

	switch (TypeOfNonLinear)
	{
		case NonLinearStatic2D::Euler:
			MyNonLinearFramePositive = this -> RunEulerMethod(NumOfIncrement, 0.01, 20, true, 
															  MyVectorTotalDisplacementPositive, 
															  MyVectorTotalExternalForcesPositive, 
															  MyVectorTotalInternalForcesPositive);
			MyNonLinearFrameNegative = this -> RunEulerMethod(NumOfIncrement, 0.01, 20, false, 
															  MyVectorTotalDisplacementNegative, 
															  MyVectorTotalExternalForcesNegative, 
															  MyVectorTotalInternalForcesNegative);
			break;

		case NonLinearStatic2D::NewtonRhapson:
			MyNonLinearFramePositive = this -> RunIncrementalIterativeMethods(NumOfIncrement, NormTolerance, 
																			  ExitCounterMax, TypeOfNonLinear, 
																			  InitialIncrementLoadRatio, true, 
																			  MyVectorTotalDisplacementPositive,
																			  MyVectorTotalExternalForcesPositive,
																			  MyVectorTotalInternalForcesPositive,
																			  MyTotalExternalForcesAtDOFPositive, 
																			  MyTotalInternalForcesAtDOFPositive, 
																			  MyFinalLoadRatiosPositive, 
																			  MyHingingHistoryPositive);
			MyNonLinearFrameNegative = this -> RunIncrementalIterativeMethods(NumOfIncrement, NormTolerance, 
																			  ExitCounterMax, TypeOfNonLinear, 
																			  InitialIncrementLoadRatio, false, 
																			  MyVectorTotalDisplacementNegative,
																			  MyVectorTotalExternalForcesNegative,
																			  MyVectorTotalInternalForcesNegative,
																			  MyTotalExternalForcesAtDOFNegative, 
																			  MyTotalInternalForcesAtDOFNegative, 
																			  MyFinalLoadRatiosNegative, 
																			  MyHingingHistoryNegative);
			break;

		case NonLinearStatic2D::ArcLength:
			MyNonLinearFramePositive = this -> RunIncrementalIterativeMethods(NumOfIncrement, NormTolerance, 
																			  ExitCounterMax, TypeOfNonLinear, 
																			  InitialIncrementLoadRatio, true, 
																			  MyVectorTotalDisplacementPositive,
																			  MyVectorTotalExternalForcesPositive,
																			  MyVectorTotalInternalForcesPositive,
																			  MyTotalExternalForcesAtDOFPositive, 
																			  MyTotalInternalForcesAtDOFPositive, 
																			  MyFinalLoadRatiosPositive,
																			  MyHingingHistoryPositive);
			MyNonLinearFrameNegative = this -> RunIncrementalIterativeMethods(NumOfIncrement, NormTolerance, 
																			  ExitCounterMax, TypeOfNonLinear, 
																			  InitialIncrementLoadRatio, false, 
																			  MyVectorTotalDisplacementNegative,
																			  MyVectorTotalExternalForcesNegative,
																			  MyVectorTotalInternalForcesNegative,
																			  MyTotalExternalForcesAtDOFNegative, 
																			  MyTotalInternalForcesAtDOFNegative, 
																			  MyFinalLoadRatiosNegative, 
																			  MyHingingHistoryNegative);
			break;
		default:
			break;
	};

	// Connect all the nonlinear static component at all the time steps when done.
	this -> ConnectAllStaticAnalysis();

	// Create the displacement vector.
	this -> SetDisplacementVector();	
	this -> SetShapeVector();
	this -> SetMassVector();
	this -> SetBeamLoadVector();
	this -> SetNodeLoadVector();
};

///<summary>Run the static analysis at load step = 0 before any loads are incremnted.</summary>
///<remarks>description</remarks>
void NonLinearStatic2D::RunStaticAnalysis()
{
	// Initialize End Forces Vector.
	this -> InitializeBeamForceVector();

	// Start a New Frame.
	//ElasticFrame2D Frame = MyBaseFrame;

	// Declare the Title.
	MyBaseFrame.SetName("Frame: Load Step = 0", 0);

	// Set the Static Loads for the Initial Frame.
	MyBaseFrame.AddNodeLoad(MyStaticNodeLoads);
	MyBaseFrame.AddBeamLoad(MyStaticMomentLoads);
	MyBaseFrame.AddBeamLoad(MyStaticPointLoads);
	MyBaseFrame.AddBeamLoad(MyStaticPyramidLoads);
	MyBaseFrame.AddBeamLoad(MyStaticTriangularLoads);
	MyBaseFrame.AddBeamLoad(MyStaticUniformLoads);

	// Solve for the displacement from the stiffness and the load vector.
	// Declare the Result Vector.
	vector <mat> StaticResults = MyBaseFrame.RunStaticAnalysis();

	// Add the external beam end forces to a running total.
	// Add the internal beam end forces to a running total.	
	this -> AddToInternalBeamEndForces(MyBaseFrame.GetInternalBeamEndForces());
	this -> AddToExternalBeamEndForces(MyBaseFrame.GetExternalBeamEndForces());
	
	MyTotalExternalForcesAtDOFPositive.push_back(MyBaseFrame.GetGlobalExternalForceVector());
	MyTotalInternalForcesAtDOFPositive.push_back(MyBaseFrame.GetGlobalInternalForceVector());
	MyFinalLoadRatiosPositive.push_back(0.0);
	MyHingingHistoryPositive.push_back(false);

	MyTotalExternalForcesAtDOFNegative.push_back(MyBaseFrame.GetGlobalExternalForceVector());
	MyTotalInternalForcesAtDOFNegative.push_back(MyBaseFrame.GetGlobalInternalForceVector());
	MyFinalLoadRatiosNegative.push_back(0.0);
	MyHingingHistoryNegative.push_back(false);

	/// TODO: Need to Make sure the static loads does not hinge the frame.  If so then must find the Hinge results.
	// Copy the base geometry to the beginning of the Analysis.
	//MyNonLinearFramePositive.push_back(Frame);
	//MyNonLinearFrameNegative.push_back(Frame);
};

///<summary>Return the Loading Direction Factor.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double NonLinearStatic2D::GetLoadingDirectionFactor(bool IsPositiveLoading)
{
	double DirectionFactor;

	if (IsPositiveLoading) 
	{
		DirectionFactor = 1.0;
	} 
	else
	{
		DirectionFactor = -1.0;
	};

	return DirectionFactor;
};

///<summary>User Euler Incremental Method, is a straight time step method to determine the response.</summary>
///<param name="IncrementFactor">Percentage of the total load to increment at each major increment.</param>
///<param name="Tolerance">Specify the Tolerance of the Hinging.</param>
///<param name="ExitCounterMax">Number of permssible loops until exit.</param>
///<remarks>description</remarks>
vector <ElasticFrame2D> NonLinearStatic2D::RunEulerMethod(int Increment, 
														  double Tolerance, 
														  int ExitCounterMax, 
														  bool IsPositiveLoading,
														  vector <mat> &VectorTotalDisplacement, 
														  vector <mat> &VectorTotalExternalForces,
														  vector <mat> &VectorTotalInternalForces)
{
	vector <ElasticFrame2D> NonLinearFrames;

	// Get the Direction Factor.
	double DirectionFactor = this -> GetLoadingDirectionFactor(IsPositiveLoading);

	// Add the base cases.
	NonLinearFrames.push_back(MyBaseFrame);

	// Time Step Counter equivalent to the Load Increment Factor.
	int MajorStep = 1;

	// Load Increment.
	int LoadStep = 1;

	// Maximumn Number of Loops allow before exiting.
	bool StayInLoop = true;

	// Get the Total Displacement.
	mat TotalDisplacements = MyBaseFrame.GetGlobalDisplacement();

	// Loop throuh the Time Step.
	while ((MajorStep <= Increment) && StayInLoop)
	{
		// Incremental Factor is used to set the overall division of the time ste.
		double IncrementFactor = DirectionFactor * 1.0 / Increment;

		// Residual Factor is declared to take see how much of the factor is left.
		double ResidualFactor = 0.0;

		// Counter to set the Max Loop for the Outside Counter.
		double OuterCounter = 0;

		// Running the Inner Loop to increment through th step.
		do
		{
			// Add to Outer Counter
			OuterCounter++;

			// Counter to set the Max Loop for the Inner Counter.
			double InnerCounter = 0;

			// Scale Factor
			double ScaleFactor = IncrementFactor - ResidualFactor;

			// Counter to Check if there is Hinging Involve.  If so then the task is exited.
			int HingingIndex = 0;

			// Declare the Result Vector.
			vector <mat> StaticResults;

			// Add the Geometry based off the previous configuration.
			ElasticFrame2D Frame = NonLinearFrames[LoadStep - 1].GetPlasticFrameGeometry();
			
			Frame.ShowNodesDisplacement = false;

			// Check if the previous loading would hinge the frame.
			Frame.CheckForHinges(VectorTotalInternalForces);

			// Declare the Title.
			Frame.SetName("Frame: Load Step = ", LoadStep);

			// Set the New Deform Shape for the Frame.
			Frame += NonLinearFrames[LoadStep - 1].GetNodeDeformation();

			// Loop to find the Hinge.
			do 
			{
				// Add to Inner Counter
				InnerCounter++;

				// Set the Incrment Node Load.
				this -> SetIncrementLoad(Frame, ScaleFactor);

				// Solve for the displacement from the stiffness and the load vector.
				StaticResults = Frame.RunStaticAnalysis(VectorTotalInternalForces);

				// Set the External Forces at the Nodes.
				mat ExternalForces = StaticResults[4] + StaticResults[6];
				
				// Display External Forces.
				if (ShowExternalEndForce) { ExternalForces.print("External End Force"); };

				// Solve for the Internal End Forces = K * u.
				mat InternalForces = StaticResults[0] * StaticResults[8];
				
				// Display Internal Forces.
				if (ShowInternalEndForce) { InternalForces.print("Internal End Forces"); };

				// Check the Moment Capacity and Release Degrees of Freedom as needed.
				Frame.CheckMomentCapacity(VectorTotalInternalForces, 
										  Frame.GetInternalBeamEndForces(),
										  Tolerance,
										  ScaleFactor,
										  HingingIndex);

				
				// Check the Loop
				if (InnerCounter > ExitCounterMax)
				{ 
					// Exit All Loop.
					StayInLoop = false;
				};

			} 
			while((HingingIndex == ElasticFrame2D::HingingIndexOverExceeded) && StayInLoop);

			// Add teh Loading to the Beam End Forces.
			this -> AddToInternalBeamEndForces(Frame.GetInternalBeamEndForces());

			// Add to the Total Displacement.
			TotalDisplacements += Frame.GetGlobalDisplacement();

			// Add the Plastic Frame if checking the Moment Capacity is good.
			NonLinearFrames.push_back(Frame);

			// Add the Scale Factor to the Residual.
			// If the Scale Factor is the same as the Increment Factor,
			// then the loop ends.
			ResidualFactor += ScaleFactor;

			// Store the Loading Increment.
			MyLoadIncrement.push_back(ScaleFactor);

			// Check the Loop
			if (OuterCounter > ExitCounterMax)
			{
				// Exit All Loop.
				StayInLoop = false;
			};

			// Increment LoadStep
			LoadStep++;
		}
		while((ResidualFactor < IncrementFactor) && StayInLoop);

		//TotalDisplacements.print("Node Displacement");

		// Increment the Time Step.
		MajorStep++;
	};

	return NonLinearFrames;
};

///<summary>Run the Incremental Iterative Methods described in McGuire, Gallagher, and Ziemians.</summary>
///<param name="NumOfIncrement">Set the max number of increment.</param>
///<param name="NormTolerance">Set the Norm and Tolerance.</param>
///<param name="ExitCounterMax"></param>
///<param name="TypeOfNonLinear">Set the type of iteration.</param>
///<param name="InitialIncrementLoadRatio">Set the Initial Incremental Load Ratio.  Usually 10%-20% of the reference load.</param>
///<remarks>description</remarks>
vector <ElasticFrame2D> NonLinearStatic2D::RunIncrementalIterativeMethods(int NumOfIncrement, 
																		  double NormTolerance, 
																		  int ExitCounterMax, 
																		  int TypeOfNonLinear, 
																		  double InitialIncrementLoadRatio, 
																		  bool IsPositiveLoading, 
																		  vector <mat> &VectorTotalDisplacement, 
																		  vector <mat> &VectorTotalExternalForces,
																		  vector <mat> &VectorTotalInternalForces,
																		  vector <mat> &TotalExternalForcesAtDOF,
																		  vector <mat> &TotalInternalForcesAtDOF,
																		  vector <double> &FinalLoadRatios, 
																		  vector <bool> &HingingHistory)
{
	vector <ElasticFrame2D> NonLinearFrames;

	// Find the Direction Factor.
	double DirectionFactor = this -> GetLoadingDirectionFactor(IsPositiveLoading);

	// Set the Base Frame.
	NonLinearFrames.push_back(MyBaseFrame);

	// Increment starts at 1 because the static load has already been applied.
	double Increment = 1;

	// Maximumn Number of Loops allow before exiting.
	bool IsNotSingular = true;

	// Get the Total Displacement.
	mat TotalDisplacements = MyBaseFrame.GetGlobalDisplacement();

	// Set the Initial Load Ratio vector and the final load ratio.
	vector <double> InitialLoadRatios;

	// Add the initial load ratio, typically between 10 - 20%.
	InitialLoadRatios.push_back(DirectionFactor * InitialIncrementLoadRatio);

	// Loop through the increment step.
	while ((Increment <= NumOfIncrement) && IsNotSingular)
	{
		// Counter.
		int Iteration = 1;

		// Loop counter to exit if iteration converged.
		bool IncrementNotConverged = true;

		vector <mat> TotalExternalForces;
		vector <mat> TotalInternalForces;
		vector <mat> IterationDisplacements;
		vector <double> LoadRatios;

		// Set the Frame to the system before it.
		TotalExternalForces.push_back(TotalExternalForcesAtDOF.back());
		TotalInternalForces.push_back(TotalInternalForcesAtDOF.back());
		
		// Declare the vector if incremental total internal forces.
		vector <mat> IncrementTotalInternalForces = VectorTotalInternalForces;

		// Store the plastic frame geometry.
		ElasticFrame2D GeometryFrame = NonLinearFrames.back().GetPlasticDeformFrameGeometry();

		// Store the ID.
		GeometryFrame.SetID(Increment);

		// Show the Nodes Displacement.
		GeometryFrame.ShowNodesDisplacement = false;

		// Declare the Residual Force Vector.
		mat TotalResidualForce;

		// Declare the Increment Displacement.
		mat IncrementDisplacements;

		// Add the Load Ratios together to find the correct increment.
		double IncrementFinalLoadRatio = 0.0;

		// Initialize the Residual Force.
		do
		{
			// Add the Geometry based off the previous configuration.
			ElasticFrame2D PrimaryFrame;
			ElasticFrame2D ResidualFrame;
			
			// Declare the Primary and Residual Internal Forces.
			vector <mat> PrimaryInternalForces;
			vector <mat> ResidualInternalForces;

			// Decare the Global, Primary, and Residual Displacement.
			mat CycleDisplacements;

			mat PrimaryDisplacement;
			mat ResidualDisplacement;

			mat TotalExternalForce;
			mat TotalInternalForce;

			// Initial guess at the load ratio; possible selection of the load ratio is possible.
			double LoadRatio;

			// Find the Residual Force and displacement.
			if (Iteration == 1)
			{
				// For the first increment the residual displacement is zero.
				// Set an empty residual displacement for the initial iteration.
				ResidualDisplacement = mat(TotalDisplacements.n_rows, 
					                       TotalDisplacements.n_cols);
				
				// Initialize to 0.0.
				ResidualDisplacement.fill(0.0);

				// Set the Increment Displacements.
				
				IncrementDisplacements = mat(TotalDisplacements.n_rows, 
					                         TotalDisplacements.n_cols);

				IncrementDisplacements.fill(0.0);
			}
			else
			{
				// Add the Geometry based off the previous configuration.
				ResidualFrame = GeometryFrame;

				// Run the Static Analysis on the Residual Force.
				ResidualFrame.RunStaticAnalysisOnResidualForce(IncrementTotalInternalForces, 
															   TotalResidualForce);

				// Find the Residual Dispalcement from the previous frame increment.
				ResidualDisplacement = ResidualFrame.GetGlobalDisplacement();

				// Find the Residual Internal Forces.
				ResidualInternalForces = ResidualFrame.GetInternalBeamEndForces();
			};
			
			// Set the Primay Frame with the geometry frame.
			PrimaryFrame = GeometryFrame;

			// Set the Incremental Load onto the frame.
			this -> SetIncrementLoad(PrimaryFrame, 1.0);

			// Solve for the displacement from the stiffness and the load vector.
			vector <mat> StaticResults = PrimaryFrame.RunStaticAnalysis(IncrementTotalInternalForces);

			// Find the primary displacement.
			PrimaryDisplacement = PrimaryFrame.GetGlobalDisplacement();
			
			// Find the internal primay forces.
			PrimaryInternalForces = PrimaryFrame.GetInternalBeamEndForces();

			// Find the appropriate Load Ratio.
			if (Iteration == 1)
			{
				// Get the Initial Load Ratio
				LoadRatio = InitialLoadRatios.back();
			}
			else
			{
				switch(TypeOfNonLinear)
				{
					case NonLinearStatic2D::NewtonRhapson:
						// Get the Load Ratio for the Newton Rhapson Method.
						LoadRatio = this -> GetLoadRatioNewtonRhapsonMethod();
						break;

					case NonLinearStatic2D::ArcLength:
						// Get the Load Ratio for the Arc Length Method.
						LoadRatio = this -> GetLoadRatioConstantArcLengthMethod(LoadRatios.front(), 
																			    IterationDisplacements.front(), 
																			    PrimaryDisplacement, 
																			    ResidualDisplacement);
						break;
				};
			};
			
			// Set the Internal Beam End Forces
			for (size_t j = 0; j < IncrementTotalInternalForces.size(); j++)
			{
				IncrementTotalInternalForces[j] += LoadRatio * PrimaryInternalForces[j];

				/// TODO: for now we leave this out.
				if (Iteration != 1 && true)
				{
					IncrementTotalInternalForces[j] += ResidualInternalForces[j];
				};
			};

			// Find the Global Displacements.
			if (Iteration == 1)
			{
				CycleDisplacements = LoadRatio * PrimaryDisplacement;
			}
			else
			{
				CycleDisplacements = LoadRatio * PrimaryDisplacement + ResidualDisplacement;
			};

			// Add to the Total Displacement.
			IncrementDisplacements += CycleDisplacements;
			TotalDisplacements += CycleDisplacements;

			// Find the External Forces at the Nodes from the primary load.
			TotalExternalForce = TotalExternalForces.back() + LoadRatio * PrimaryFrame.GetGlobalExternalForceVector();

			TotalInternalForce = TotalInternalForces.back() + LoadRatio * PrimaryFrame.GetGlobalInternalForceVector();

			// Find the Internal Forces at the Nodes.
			if (Iteration != 1)
			{
				// Return the Residual Internal Force.
				mat ResidualInternalForce = ResidualFrame.GetGlobalInternalForceVector();
				
				// Add to the total internal force. 
				TotalInternalForce +=  ResidualInternalForce;
			};

			// Total Residual Force = External Force - Internal Force				
			TotalResidualForce = TotalExternalForce - TotalInternalForce;
			
			// Print the Internal Forces.
			if (false)
			{
				//ResidualInternalForce.print("Residual Internal Force");
				//TotalExternalForce.print("External Force");
				//TotalInternalForce.print("Internal Force");
				//TotalResidualForce.print("Residual Force");
			};
			
			// Set the Deform Nodes.
			GeometryFrame.SetNodesDisplacements(IncrementDisplacements);

			// Store the Load Ratio, Global Displacement, Primary Frame, Residual Frame, and Total Incremental Load Rati.
			LoadRatios.push_back(LoadRatio);
			IterationDisplacements.push_back(CycleDisplacements);
			TotalExternalForces.push_back(TotalExternalForce);
			TotalInternalForces.push_back(TotalInternalForce);
			IncrementFinalLoadRatio += LoadRatio;	

			// Norm Tolerance.
			if (this -> GetNormModifiedEuclidean(CycleDisplacements, TotalDisplacements) < NormTolerance)
			{
				// Exit Loop.
				IncrementNotConverged = false;
			};

			//PrimaryFrame.Print();

			// Counter
			Iteration++;
		}
		while(IncrementNotConverged && Iteration < ExitCounterMax);

		// Return Index used to set the information on the correct algorithm.
		int HingingIndex = -1;

		vector <mat> DeltaInternalForces;

		for (int k = 0; k < IncrementTotalInternalForces.size(); k++)
		{
			DeltaInternalForces.push_back(IncrementTotalInternalForces[k] - VectorTotalInternalForces[k]);
		};
		
		// Turn off outputing tables results.
		GeometryFrame.ShowRatioTables = false;

		// Now we check for hinges.  If it hinges, we change the initial load ratio and start over.
		GeometryFrame.CheckMomentCapacity(VectorTotalInternalForces, 
										  DeltaInternalForces,
										  0.01,
										  IncrementFinalLoadRatio,
										  HingingIndex);
		
		// Determine the Total Load Ratio.
		double TotalLoad = 0.0;

		// ********************************
		// Depending on the outcome of the hinging index. The choice is to either keep the load ratio, decrease the load ratio, 
		// or determine that the load ratio is at a hinging location and then the hinge is set and save the load ratio.
		// ********************************
		switch (HingingIndex)
		{
			case ElasticFrame2D::HingingIndexNotExceeded:
				// *********************************
				// Hinging is not exceeded, therefore the load ratio is save and the automatic load increment is utilized)
				// *********************************

				// Find the next initial load ratio.
				InitialLoadRatios.push_back(this -> GetAutomaticLoadIncrementation(InitialLoadRatios.back(), 
																				   Iteration, 
																				   5, 
																				   0.5, 
																				   0.25));

				// Store the last setting of the frame geometry, total external forces at DOF, and total internal forces at DOF.
				NonLinearFrames.push_back(GeometryFrame);
				TotalExternalForcesAtDOF.push_back(TotalExternalForces.back());
				TotalInternalForcesAtDOF.push_back(TotalInternalForces.back());

				// Store the final load ratios for this increment step.
				FinalLoadRatios.push_back(IncrementFinalLoadRatio);

				// Change to the new internal load ratio.
				VectorTotalInternalForces = IncrementTotalInternalForces;

				// No Hinging.
				HingingHistory.push_back(false);

				// Change the Increment Factor.
				Increment++;

				break;

			case ElasticFrame2D::HingingIndexOverExceeded:
				// *********************************
				// Hinging is exceeded, therefore the load ratio is reduced and save over the last increment.
				// *********************************

				// Save over the previous load ratio and loop again.
				InitialLoadRatios.back() = IncrementFinalLoadRatio;

				break;

			case ElasticFrame2D::HingingIndexConverged:
				// Change the Total Displacements to account for the hinges.

				// Find the next initial load ratio.
				InitialLoadRatios.push_back(this -> GetAutomaticLoadIncrementation(InitialLoadRatios.back(), 
																				   Iteration, 
																				   5, 
																				   0.5, 
																				   0.25));

				// Store the last setting of the frame geometry, total external forces at DOF, and total internal forces at DOF.
				NonLinearFrames.push_back(GeometryFrame);

				// Set the Hinging Matrix.
				int InitialIndex = NonLinearFrames.size() - 2;
				int FinalIndex = NonLinearFrames.size() - 1;

				mat HingingMatrix = this -> GetHingingPermutationMatrix(NonLinearFrames[InitialIndex], 
																		NonLinearFrames[FinalIndex]);

				// Transform the Displacements Vector to add an additional DOF.
				TotalDisplacements =  HingingMatrix.t() * TotalDisplacements;

				TotalExternalForcesAtDOF.push_back(HingingMatrix.t() * TotalExternalForces.back());
				TotalInternalForcesAtDOF.push_back(HingingMatrix.t() * TotalInternalForces.back());

				// Transform the Displacements Vector for the non-linear frame. (TEMPORARY - This should actually save the non-linear frame without the hinging first, and then hinge the primary frame);
				NonLinearFrames.back().ModifyVectorForHinges(HingingMatrix);

				// Store the final load ratios for this increment step.
				FinalLoadRatios.push_back(IncrementFinalLoadRatio);

				// Change to the new internal load ratio.
				VectorTotalInternalForces = IncrementTotalInternalForces;

				// Hinging.
				HingingHistory.push_back(true);

				Increment++;

				break;
		};

		// Add the Loads Ratio together.
		for (size_t k = 0; k < FinalLoadRatios.size(); k++) { TotalLoad += FinalLoadRatios[k]; };

		//cout << "Total Load\n" << TotalLoad << endl;

		//TotalDisplacements.print("Total Displacements");
	};

	// Set the external loads onto the frame.
	for (size_t i = 1; i < NonLinearFrames.size(); i++)
	{
		// Establish the Incremental Loading from the final load condition.
		this -> SetIncrementLoad(NonLinearFrames[i], FinalLoadRatios[i]);
		
		// Create the vectors.
		NonLinearFrames[i].RunStaticAnalysis(false);
	};

	return NonLinearFrames;
};

///<summary>This algorithm utilizes the unmodified Newton-Raphson technique or the Load Control Method.
/// As stated in 'Matrix Structural Analysis, 2nd Edition', the method utilizes "a fixed amount of load 
/// employed at each increment.  All of the load is applied at the first step j = 1 and additional iteration 
/// j >= 2 are only performin an attempt to satisfy equilibrium requirements.
/// See McGuire page 346 for explanation.</summary>
///<param name="Increment">Set the number of incrmentation of the newton rhapson method.</param>
///<param name="Tolerance">Set the tolerance of the method.</param>
///<param name="ExitCounterMax">Set the maximum number of iteration between you exit counter.<param>
///<remarks>description</remarks>
double NonLinearStatic2D::GetLoadRatioNewtonRhapsonMethod()
{
	/*

	   /|\
		|								 ...............
	____|____________________________......
		|			/	/  /	.....
		|          / ! / !/ !.....
		|         /  !/   ...
		|        /   ...
	dF	|       / ..
		|      /..
		|     /.
		|    /.
	____|__[.]
		|  .
		| .	
		|.  |		|	|			  | 
		 ------------------------------------------------------->
		    |		|	|			  |
			Ui    Ui_1
	*/

	return 0.0;
};

///<summary>Return the constant arc length method load ratio scale factor 
/// as described in McGuire book.</summary>
///<param name="InitialLoadRatio">Initial load ratio guess at iteration = 1.</param>
///<param name="DisplacementInitial">Initial displacement vector at iteration = 1.</param>
///<param name="DisplacementReferenceLoad">Displacement vector at this iteration.</param>
///<param name="DisplacementResidualLoad">Dispalcement vector due to the residual load at this iteration.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double NonLinearStatic2D::GetLoadRatioConstantArcLengthMethod(double InitialLoadRatio, 
															  mat DisplacementInitial,
															  mat DisplacementReferenceLoad, 
															  mat DisplacementResidualLoad)
{
	// Initialize the Load Ratio.
	double LoadRatio = 0.0;

	mat Numerator = DisplacementInitial.t() * DisplacementResidualLoad;

	mat Denominator = DisplacementInitial.t() * DisplacementReferenceLoad + InitialLoadRatio;

	LoadRatio = - Numerator(0, 0) / Denominator(0, 0);

	return LoadRatio;
};

///<summary>Return the Modified Euclidean Norm value.</summary>
///<param name="IterationDisplacement"></param>
///<param name="TotalDisplacement">Find the total displacements.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double NonLinearStatic2D::GetNormModifiedEuclidean(arma::mat IterationDisplacement, 
												   arma::mat TotalDisplacement)
{
	double Norm = 0.0;

	// First find the maximum displacment.
	double DeltaReference;

	double MaxValue = TotalDisplacement.max();
	double MinValue = TotalDisplacement.min();

	if (abs(MinValue) < abs(MaxValue))
	{
		DeltaReference = MaxValue;
	}
	else
	{
		DeltaReference = MinValue;
	};

	// The summation of the displacement ratio.
	double SumOfRatio = 0.0;

	for (size_t k = 0; k < IterationDisplacement.size(); k++)
	{
		SumOfRatio += pow(IterationDisplacement[k] / DeltaReference, 2);
	};

	Norm = sqrt(SumOfRatio / IterationDisplacement.size());

	return Norm;
};

///<summary>After the incremental analysis is done, than it necessary to reconnect all nodes, elements, and loads.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>This is nessesary because when using 'vector' data type, every time it pushes back it does a recopy of
/// all the cells before it - losing the connections.</remarks>
void NonLinearStatic2D::ConnectAllStaticAnalysis()
{
	// Loop through each Plastic Frame to connect the component.
	for (size_t i = 0; i < MyNonLinearFramePositive.size(); i++)
	{
		MyNonLinearFramePositive[i].ConnectComponents();
	};

	for (size_t j = 0; j < MyNonLinearFrameNegative.size(); j++)
	{
		MyNonLinearFrameNegative[j].ConnectComponents();
	};
};

#pragma endregion

///<summary>Add the external beam end forces to the vector.</summary>
///<param name="BeamEndForces">A vector of the beam end force matrix.</param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddToExternalBeamEndForces(std::vector<mat> BeamEndForces)
{
	// Loop and add the Increment.
	for (size_t j = 0; j < BeamEndForces.size(); j++)
	{
		// Store the Local Forces.
		MyVectorTotalExternalForcesPositive[j] += BeamEndForces[j];
		MyVectorTotalExternalForcesNegative[j] += BeamEndForces[j];
	};
};


///<summary>Add the internal beam end forces to the vector.</summary>
///<param name="BeamEndForces">A vector of the beam end force matrix.</param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddToInternalBeamEndForces(vector <mat> BeamEndForces)
{
	// Loop and add the Increment.
	for (size_t j = 0; j < BeamEndForces.size(); j++)
	{
		// Store the Local Forces.
		MyVectorTotalInternalForcesPositive[j] += BeamEndForces[j];
		MyVectorTotalInternalForcesNegative[j] += BeamEndForces[j];
	};
};


///<summary>Return the Nodal Load History of a Node.</summary>
///<param name="ID"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <Node2D> NonLinearStatic2D::GetNodeLoadHistory(int ID, 
													  bool IsPositiveLoading)
{
	// An array of Nodes.
	vector <Node2D> Nodes;

	// Loop through getting the Node Displacement.
	if (IsPositiveLoading)
	{
		for (size_t i = 0; i < MyNonLinearFramePositive.size(); i++)
		{
			Nodes.push_back(MyNonLinearFramePositive[i].GetNodeDisplacement(ID));
		};
	}
	else
	{
		for (size_t i = 0; i < MyNonLinearFrameNegative.size(); i++)
		{
			Nodes.push_back(MyNonLinearFrameNegative[i].GetNodeDisplacement(ID));
		};
	};

	// Return Nodes.
	return Nodes;
};

///<summary>Initialize the beam end force vector to zeros.</summary>
///<remarks>description</remarks>
void NonLinearStatic2D::InitializeBeamForceVector()
{
	// Initialize Vector
	for (size_t i = 0; i < MyBaseFrame.GetNumberOfBeams(); i++)
	{
		// Declare Beam Local Force Array.
		mat BeamEndForces(6, 1);

		// Initialize Local Force Array.
		BeamEndForces.fill(0.0);

		// Add to the Vector.
		MyVectorTotalExternalForcesPositive.push_back(BeamEndForces);
		MyVectorTotalInternalForcesPositive.push_back(BeamEndForces);

		MyVectorTotalExternalForcesNegative.push_back(BeamEndForces);
		MyVectorTotalInternalForcesNegative.push_back(BeamEndForces);
	};
};

///<summary>Return the automatic load incrementation as implement in McGuire page 352.</summary>
///<param name="PreviousLoadRatio">The previous increment load ratio.</param>
///<param name="PreviousIteration">The previous number of iteration.</param>
///<param name="DesiredIteration">The desired number of iteration.</param>
///<param name="ExponentParameter">The exponental parameter, typically 0.5 - 1.0.</param>
///<param name="MaxLoadIncrement">Maximum allowable load ratio.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double NonLinearStatic2D::GetAutomaticLoadIncrementation(double PreviousLoadRatio, 
														 double PreviousIteration, 
														 double DesiredIteration, 
														 double ExponentParameter, 
														 double MaxLoadRatio)
{
	//
	double Gamma = DesiredIteration / PreviousIteration;

	// Find the Load Ratio.
	double LoadRatio = PreviousLoadRatio * pow(abs(Gamma), ExponentParameter);

	// Verify that the load ratio does not go beyond the maximum allowable.
	if (LoadRatio > MaxLoadRatio)
	{
		LoadRatio = MaxLoadRatio;
	}

	return LoadRatio;
};

///<summary>Return the permutation matrix when the frame hinges.</summary>
///<param name="InitialIndex">Initial Frame before hinging..</param>
///<param name="FinalIndex">Final Frame after hinging.</param>
///<returns>description</returns>
///<remarks>description</remarks>
mat NonLinearStatic2D::GetHingingPermutationMatrix(ElasticFrame2D &InitialFrame,
												   ElasticFrame2D &FinalFrame)
{
	InitialFrame.ConnectComponents();
	FinalFrame.ConnectComponents();

	// Get the Matrix Size.
	mat HingingMatrix(InitialFrame.GetDofIndex(), 
					  FinalFrame.GetDofIndex());

	HingingMatrix.fill(0.0);

	// Get the Initial and Final Nodes.
	vector <Node2D> InitialNodes = InitialFrame.GetNodes();
	vector <Node2D> FinalNodes = FinalFrame.GetNodes();

	// Return the Nodal Index List.
	for (size_t i = 0; i < InitialNodes.size(); i++)
	{
		// Return the Initial Node List Index.
		vector <int> InitialNodeList = InitialNodes[i].GetNodeDOFIndexes();

		// Return the Final Node List Index.
		vector <int> FinalNodeList = FinalNodes[i].GetNodeDOFIndexes();

		for (size_t j = 0; j < 3; j++)
		{
			switch (j)
			{
				case 0:
					// Get the x degree of freedom.
					InitialNodeList = InitialNodes[i].GetDOFIndexX();
					FinalNodeList = FinalNodes[i].GetDOFIndexX();
					break;
				case 1:
					// Get the y degree of freedom.
					InitialNodeList = InitialNodes[i].GetDOFIndexY();
					FinalNodeList = FinalNodes[i].GetDOFIndexY();
					break;
				case 2:
					// Get the z degree of freedom.
					InitialNodeList = InitialNodes[i].GetDOFIndexZ();
					FinalNodeList = FinalNodes[i].GetDOFIndexZ();
					break;
			};

			// Add 1.0 correlating the matrix.
			for (size_t k = 0; k < InitialNodeList.size(); k++)
			{
				// Set the new increment.
				HingingMatrix(InitialNodeList[k], 
					          FinalNodeList[k]) = 1.0;
			};	
		};		
	};
	
	// Show Results.
	if (false) { HingingMatrix.print("Hinging Matrix"); };
	
	// Return the Matrix.
	return HingingMatrix;
};

#pragma region "Setting Geometry and Loads"

///<summary>Set the Geometry of the Plastic Frame.</summary>
///<param name="Frame"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetFrameGeometry(ElasticFrame2D Frame)
{
	MyBaseFrame = Frame;
};

///<summary>Return the Frame Geometry.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
ElasticFrame2D NonLinearStatic2D::GetFrameGeometry()
{
	return MyBaseFrame;
};

///<summary>Add the Static Node Load.</summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(NodeLoad2D Load)
{
	// Node ID
	int NodeID = Load.GetNodeID();

	// Find the Index.
	int Index = MyBaseFrame.GetNodeIndex(NodeID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetNode(&MyBaseFrame.GetNodeAddress(NodeID));
		
		// Save the Load.
		MyStaticNodeLoads.push_back(Load);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(MomentLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyStaticMomentLoads.push_back(Load);
	};
};

/// <summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(PointLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyStaticPointLoads.push_back(Load);
	};
	
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(PyramidLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyStaticPyramidLoads.push_back(Load);
	};
	
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(TriangularLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyStaticTriangularLoads.push_back(Load);
	};
	
};

/// <summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(UniformLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyStaticUniformLoads.push_back(Load);
	};
	
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(std::vector<NodeLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddStaticLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(std::vector<MomentLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddStaticLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(std::vector<PointLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddStaticLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(std::vector<PyramidLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddStaticLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(std::vector<TriangularLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddStaticLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddStaticLoad(std::vector<UniformLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddStaticLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(NodeLoad2D Load)
{
	// Node ID
	int NodeID = Load.GetNodeID();

	// Find the Index.
	int Index = MyBaseFrame.GetNodeIndex(NodeID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetNode(&MyBaseFrame.GetNodeAddress(NodeID));

		// Save the Load.
		MyIncrementNodeLoads.push_back(Load);
	};
	
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(MomentLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyIncrementMomentLoads.push_back(Load);
	};
	
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(PointLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyIncrementPointLoads.push_back(Load);
	};
	
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(PyramidLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyIncrementPyramidLoads.push_back(Load);
	};
	
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(TriangularLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyIncrementTriangularLoads.push_back(Load);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(UniformLoad2D Load)
{
	// Beam ID
	int BeamID = Load.GetBeamID();

	// Find the Index.
	int Index = MyBaseFrame.GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyBaseFrame.GetBeamAddress(BeamID));

		// Save the Load.
		MyIncrementUniformLoads.push_back(Load);
	};
	
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(vector <NodeLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddIncrementalLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(vector <MomentLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddIncrementalLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(vector <PointLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddIncrementalLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(vector <PyramidLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddIncrementalLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(vector <TriangularLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddIncrementalLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::AddIncrementalLoad(vector <UniformLoad2D> Load)
{
	// Loop and add the Load.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddIncrementalLoad(Load[i]);
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetIncrementNodeLoads(ElasticFrame2D &Frame, double LoadFactor)
{
	// Decalare a vector of Nodal Loads.
	vector <NodeLoad2D> Loads;

	// Set the Incremental Nodal Load for the specific time step.
	for (size_t i = 0; i < MyIncrementNodeLoads.size(); i++)
	{
		// Copy the Content of the Node Load to a new instance.
		NodeLoad2D Load = MyIncrementNodeLoads[i];

		// Return the Nodal Load Increment for the corresponding Nodal Load.
		Loads.push_back(Load * LoadFactor);
	};

	// Add the Nodal Loads.
	Frame.AddNodeLoad(Loads);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetIncrementLoad(ElasticFrame2D &Frame, double LoadFactor)
{
	// Clear all the Loads before Setting Incrementation.
	Frame.ClearLoads();

	// Set All the Loads.
	this -> SetIncrementNodeLoads(Frame, LoadFactor);
	this -> SetIncrementMomentLoad(Frame, LoadFactor);
	this -> SetIncrementPointLoad(Frame, LoadFactor);
	this -> SetIncrementPyramidLoad(Frame, LoadFactor);
	this -> SetIncrementTriangularLoad(Frame, LoadFactor);
	this -> SetIncrementUniformLoad(Frame, LoadFactor);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetIncrementMomentLoad(ElasticFrame2D &Frame, double LoadFactor)
{
	// Declare the Moment Load Vector.
	vector <MomentLoad2D> Loads;

	// Set the Incremental Moment Load Vector for the specific time step.
	for (size_t i = 0; i < MyIncrementMomentLoads.size(); i++)
	{
		// Copy the Content of the Moment Load to a new instance for changes.
		MomentLoad2D Load = MyIncrementMomentLoads[i];

		// Return the Load Increment.
		Loads.push_back(Load * LoadFactor);
	};

	// Add the Nodal Loads.
	Frame.AddBeamLoad(Loads);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetIncrementPointLoad(ElasticFrame2D &Frame, 
											  double LoadFactor)
{
	// Declare the Moment Load Vector.
	vector <PointLoad2D> Loads;

	// Set the Incremental Moment Load Vector for the specific time step.
	for (size_t i = 0; i < MyIncrementPointLoads.size(); i++)
	{
		// Copy the content of the Point Load to a new instance for changes.
		PointLoad2D Load = MyIncrementPointLoads[i];

		// Return the Load Increment.
		Loads.push_back(Load * LoadFactor);
	};

	// Add the Nodal Loads.
	Frame.AddBeamLoad(Loads);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetIncrementPyramidLoad(ElasticFrame2D &Frame, 
												double LoadFactor)
{
	// Declare the Moment Load Vector.
	vector <PyramidLoad2D> Loads;

	// Set the Incremental Moment Load Vector for the specific time step.
	for (size_t i = 0; i < MyIncrementPyramidLoads.size(); i++)
	{
		// Copy the content of the Pyramid Load to a new instance for changing.
		PyramidLoad2D Load = MyIncrementPyramidLoads[i];

		// Return the Load Increment.
		Loads.push_back(Load * LoadFactor);
	};

	// Add the Nodal Loads.
	Frame.AddBeamLoad(Loads);
};

///<summary>Increment all the Triangular Load in the frame.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetIncrementTriangularLoad(ElasticFrame2D &Frame, 
												   double LoadFactor)
{
	// Declare the Moment Load Vector.
	vector <TriangularLoad2D> Loads;

	// Set the Incremental Moment Load Vector for the specific time step.
	for (size_t i = 0; i < MyIncrementTriangularLoads.size(); i++)
	{
		// Copy the content of the Triangular Load to a new instance for changing.
		TriangularLoad2D Load = MyIncrementTriangularLoads[i];

		// Return the Load Increment.
		Loads.push_back(Load * LoadFactor);
	};

	// Add the Nodal Loads.
	Frame.AddBeamLoad(Loads);
};

///<summary>Increment all the Uniform Loads in the Frame.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetIncrementUniformLoad(ElasticFrame2D &Frame, 
												double LoadFactor)
{
	// Declare the Moment Load Vector.
	vector <UniformLoad2D> Loads;

	// Set the Incremental Moment Load Vector for the specific time step.
	for (size_t i = 0; i < MyIncrementUniformLoads.size(); i++)
	{
		// Copy the content of the Uniform Load to a new instance for changing.
		UniformLoad2D Load = MyIncrementUniformLoads[i];

		// Return the Load Increment.
		Loads.push_back(Load * LoadFactor);
	};

	// Add the Nodal Loads.
	Frame.AddBeamLoad(Loads);
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <NodeLoad2D> NonLinearStatic2D::GetStaticNodeLoad2D()
{
	return MyStaticNodeLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <MomentLoad2D> NonLinearStatic2D::GetStaticMomentLoad2D()
{
	return MyStaticMomentLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <PointLoad2D> NonLinearStatic2D::GetStaticPointLoad2D()
{
	return MyStaticPointLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <PyramidLoad2D> NonLinearStatic2D::GetStaticPyramidLoad2D()
{
	return MyStaticPyramidLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <TriangularLoad2D> NonLinearStatic2D::GetStaticTriangularLoad2D()
{
	return MyStaticTriangularLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <UniformLoad2D> NonLinearStatic2D::GetStaticUniformLoad2D()
{
	return MyStaticUniformLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <NodeLoad2D> NonLinearStatic2D::GetIncrementalNodeLoad2D()
{
	return MyIncrementNodeLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <MomentLoad2D> NonLinearStatic2D::GetIncrementalMomentLoad2D()
{
	return MyIncrementMomentLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <PointLoad2D> NonLinearStatic2D::GetIncrementalPointLoad2D()
{
	return MyIncrementPointLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <PyramidLoad2D> NonLinearStatic2D::GetIncrementalPyramidLoad2D()
{
	return MyIncrementPyramidLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <TriangularLoad2D> NonLinearStatic2D::GetIncrementalTriangularLoad2D()
{
	return MyIncrementTriangularLoads;
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <UniformLoad2D> NonLinearStatic2D::GetIncrementalUniformLoad2D()
{
	return MyIncrementUniformLoads;
}

#pragma endregion 

#pragma region "Resistance Function"

///<summary>Set the displacement vector for each time history.</summary>
///<remarks>description</remarks>
void NonLinearStatic2D::SetDisplacementVector()
{
	// ================== STORE THE DISPLACEMENT VECTOR AT EACH LOAD STEP =======================
	// ****************************** POSITIVE **************************************************
	
	// LoadStep
	vector <double> TotalTranslationX;
	vector <double> TotalTranslationY;
	vector <double> TotalRotationZ;

	for (int i = 0; i < MyNonLinearFramePositive.size(); i++)
	{
		vector <double> DeltaTranslationX;
		vector <double> DeltaTranslationY;
		vector <double> DeltaRotationZ;

		if (i == 0)
		{
			// Set the Translation X.
			TotalTranslationX = MyNonLinearFramePositive[i].GetDisplacementVector(NonLinearStatic2D::TranslationX);

			// Set the Translation Y.
			TotalTranslationY = MyNonLinearFramePositive[i].GetDisplacementVector(NonLinearStatic2D::TranslationY);

			// Set the Rotation Z.
			//TotalRotationZ = MyNonLinearFramePositive[i].GetDisplacementVector(NonLinearStatic2D::RotationZ);
		}
		else
		{
			// Set the Translation X.
			vector <double> IncrementTranslationX = MyNonLinearFramePositive[i].GetDisplacementVector(NonLinearStatic2D::TranslationX);

			// Set the Translation Y.
			vector <double> IncrementTranslationY = MyNonLinearFramePositive[i].GetDisplacementVector(NonLinearStatic2D::TranslationY);

			// Set the Rotation Z.
			vector <double> IncrementRotationZ = MyNonLinearFramePositive[i].GetDisplacementVector(NonLinearStatic2D::RotationZ);

			// Add the Translation X.
			for (size_t h = 0; h < IncrementTranslationX.size(); h++)
			{
				TotalTranslationX[h] += IncrementTranslationX[h];
			};

			// Add Translation Y.
			for (size_t j = 0; j < IncrementTranslationY.size(); j++)
			{
				TotalTranslationY[j] += IncrementTranslationY[j];
			};

			// Add Rotation Z.
			for (size_t k = 0; k < IncrementRotationZ.size(); k++)
			{
				//TotalRotationZ[k] += IncrementRotationZ[k];
			};
		}

		// Store the Displacements in this Vector.
		MyLocationVectorPositiveTranslationX.push_back(TotalTranslationX);
		MyLocationVectorPositiveTranslationY.push_back(TotalTranslationY);
		//MyLocationVectorPositiveRotationZ.push_back(TotalRotationZ);

		// Find the Difference between the applied static load and the incremental load.
		// Add the Translation X.
		for (size_t h = 0; h < TotalTranslationX.size(); h++)
		{
			DeltaTranslationX.push_back(TotalTranslationX[h] - MyLocationVectorPositiveTranslationX[0][h]);
		};

		// Add Translation Y.
		for (size_t j = 0; j < TotalTranslationY.size(); j++)
		{
			DeltaTranslationY.push_back(TotalTranslationY[j] - MyLocationVectorPositiveTranslationY[0][j]);
		};

		// Find the displacement from the static position.
		for (size_t k = 0; k < TotalRotationZ.size(); k++)
		{
			//DeltaRotationZ.push_back(TotalRotationZ[k] - MyLocationVectorPositiveRotationZ[0][k]);
		};

		MyDisplacementVectorPositiveTranslationX.push_back(DeltaTranslationX);
		MyDisplacementVectorPositiveTranslationY.push_back(DeltaTranslationY);
		//MyDisplacementVectorPositiveRotationZ.push_back(DeltaRotationZ);
	};

	// ****************************** NEGATIVE **************************************************
	// LoadStep
	TotalTranslationX.clear();
	TotalTranslationY.clear();
	//TotalRotationZ.clear();

	for (int i = 0; i < MyNonLinearFrameNegative.size(); i++)
	{
		vector <double> DeltaTranslationX;
		vector <double> DeltaTranslationY;
		vector <double> DeltaRotationZ;

		if (i == 0)
		{
			
			// Set the Translation X.
			TotalTranslationX = MyNonLinearFrameNegative[i].GetDisplacementVector(NonLinearStatic2D::TranslationX);

			// Set the Translation Y.
			TotalTranslationY = MyNonLinearFrameNegative[i].GetDisplacementVector(NonLinearStatic2D::TranslationY);

			// Set the Rotation Z.
			//TotalRotationZ = MyNonLinearFrameNegative[i].GetDisplacementVector(NonLinearStatic2D::RotationZ);
		}
		else
		{
			// Set the Translation X.
			vector <double> IncrementTranslationX = MyNonLinearFrameNegative[i].GetDisplacementVector(NonLinearStatic2D::TranslationX);

			// Set the Translation Y.
			vector <double> IncrementTranslationY = MyNonLinearFrameNegative[i].GetDisplacementVector(NonLinearStatic2D::TranslationY);

			// Set the Rotation Z.
			//vector <double> IncrementRotationZ = MyNonLinearFrameNegative[i].GetDisplacementVector(NonLinearStatic2D::RotationZ);

			// Add the Translation X.
			for (size_t h = 0; h < IncrementTranslationX.size(); h++)
			{
				TotalTranslationX[h] += IncrementTranslationX[h];
			};

			// Add Translation Y.
			for (size_t j = 0; j < IncrementTranslationY.size(); j++)
			{
				TotalTranslationY[j] += IncrementTranslationY[j];
			};

			// Add Rotation Z.
			//for (size_t k = 0; k < IncrementRotationZ.size(); k++)
			//{
				//TotalRotationZ[k] += IncrementRotationZ[k];
			//};
		}

		// Store the Displacements in this Vector.
		MyLocationVectorNegativeTranslationX.push_back(TotalTranslationX);
		MyLocationVectorNegativeTranslationY.push_back(TotalTranslationY);
		//MyLocationVectorNegativeRotationZ.push_back(TotalRotationZ);

		// Find the Difference between the applied static load and the incremental load.
		// Add the Translation X.
		for (size_t h = 0; h < TotalTranslationX.size(); h++)
		{
			DeltaTranslationX.push_back(TotalTranslationX[h] - TotalTranslationX[0]);
		};

		// Add Translation Y.
		for (size_t j = 0; j < TotalTranslationY.size(); j++)
		{
			DeltaTranslationY.push_back(TotalTranslationY[j] - TotalTranslationY[0]);
		};

		// Find the displacement from the static position.
		for (size_t k = 0; k < TotalRotationZ.size(); k++)
		{
			//DeltaRotationZ.push_back(TotalRotationZ[k] - TotalRotationZ[0]);
		};

		MyDisplacementVectorNegativeTranslationX.push_back(DeltaTranslationX);
		MyDisplacementVectorNegativeTranslationY.push_back(DeltaTranslationY);
		//MyDisplacementVectorNegativeRotationZ.push_back(DeltaRotationZ);
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetMassVector()
{
	// *** POSITIVE ***
	for (size_t i = 0; i < MyNonLinearFramePositive.size(); i++)
	{
		// X DOF
		MyMassVectorPositiveTranslationX.push_back(MyNonLinearFramePositive[i].GetMassVector(NonLinearStatic2D::TranslationX)); 
		
		// Y DOF
		MyMassVectorPositiveTranslationY.push_back(MyNonLinearFramePositive[i].GetMassVector(NonLinearStatic2D::TranslationY));

		// Z DOF
		MyMassVectorPositiveRotationZ.push_back(MyNonLinearFramePositive[i].GetMassVector(NonLinearStatic2D::RotationZ));
	};	
	
	// *** NEGATIVE ***
	for (size_t i = 0; i < MyNonLinearFrameNegative.size(); i++)
	{
		// X DOF
		MyMassVectorNegativeTranslationX.push_back(MyNonLinearFrameNegative[i].GetMassVector(NonLinearStatic2D::TranslationX)); 
		
		// Y DOF
		MyMassVectorNegativeTranslationY.push_back(MyNonLinearFrameNegative[i].GetMassVector(NonLinearStatic2D::TranslationY));

		// Z DOF
		MyMassVectorNegativeRotationZ.push_back(MyNonLinearFrameNegative[i].GetMassVector(NonLinearStatic2D::RotationZ));
	};	
};

///<summary></summary>
///<remarks>description</remarks>
void NonLinearStatic2D::SetNodeLoadVector()
{
	// LoadStep
	vector <double> TotalTranslationX;
	vector <double> TotalTranslationY;
	vector <double> TotalRotationZ;

	// *** POSITIVE ***
	for (size_t i = 0; i < MyNonLinearFramePositive.size(); i++)
	{
		
		if (i == 0)
		{
			// Find the Number of Beams.
			size_t NumOfBeams = MyNonLinearFramePositive[i].GetNumberOfBeams();

			// Declare the Matrix.
			vector <double> InitializeLoads;

			for (size_t j = 0; j < NumOfBeams; j++)
			{
				InitializeLoads.push_back(0.0);
				InitializeLoads.push_back(0.0);
			};

			// Set the Translation X.
			TotalTranslationX = InitializeLoads;

			// Set the Translation Y.
			TotalTranslationY = InitializeLoads;

			// Set the Rotation Z.
			TotalRotationZ = InitializeLoads;
		}
		else
		{
			// Set the Translation X.
			vector <double> IncrementTranslationX = MyNonLinearFramePositive[i].GetNodeLoadVector(ElasticFrame2D::TranslationX);

			// Set the Translation Y.
			vector <double> IncrementTranslationY = MyNonLinearFramePositive[i].GetNodeLoadVector(ElasticFrame2D::TranslationY);

			// Set the Rotation Z.
			vector <double> IncrementRotationZ = MyNonLinearFramePositive[i].GetNodeLoadVector(ElasticFrame2D::RotationZ);

			// Add Translation X.
			for (size_t h = 0; h < IncrementTranslationX.size(); h++)
			{
				TotalTranslationX[h] += IncrementTranslationX[h];
			};

			// Add Translation Y.
			for (size_t j = 0; j < IncrementTranslationY.size(); j++)
			{
				TotalTranslationY[j] += IncrementTranslationY[j];
			};

			// Add Rotation Z.
			for (size_t k = 0; k < IncrementRotationZ.size(); k++)
			{
				//TotalRotationZ[k] += IncrementRotationZ[k];
			};
		}

		MyNodeLoadVectorPositiveTranslationX.push_back(TotalTranslationX);
		MyNodeLoadVectorPositiveTranslationY.push_back(TotalTranslationY);
		MyNodeLoadVectorPositiveRotationZ.push_back(TotalRotationZ);
	};

	// LoadStep
	TotalTranslationX.clear();
	TotalTranslationY.clear();
	TotalRotationZ.clear();

	// *** NEGATIVE ***
	for (size_t i = 0; i < MyNonLinearFrameNegative.size(); i++)
	{
		if (i == 0)
		{
			// Find the Number of Beams.
			size_t NumOfBeams = MyNonLinearFramePositive[i].GetNumberOfBeams();

			// Declare the Matrix.
			vector <double> InitializeLoads;

			for (size_t j = 0; j < NumOfBeams; j++)
			{
				InitializeLoads.push_back(0.0);
				InitializeLoads.push_back(0.0);
			};

			// Set the Translation X.
			TotalTranslationX = InitializeLoads;

			// Set the Translation Y.
			TotalTranslationY = InitializeLoads;

			// Set the Rotation Z.
			TotalRotationZ = InitializeLoads;
		}
		else
		{
			// Set the Translation X.
			vector <double> IncrementTranslationX = MyNonLinearFrameNegative[i].GetNodeLoadVector(ElasticFrame2D::TranslationX);

			// Set the Translation Y.
			vector <double> IncrementTranslationY = MyNonLinearFrameNegative[i].GetNodeLoadVector(ElasticFrame2D::TranslationY);

			// Set the Rotation Z.
			vector <double> IncrementRotationZ = MyNonLinearFrameNegative[i].GetNodeLoadVector(ElasticFrame2D::RotationZ);

			// Add Translation X.
			for (size_t h = 0; h < IncrementTranslationX.size(); h++)
			{
				TotalTranslationX[h] += IncrementTranslationX[h];
			};

			// Add Translation Y.
			for (size_t j = 0; j < IncrementTranslationY.size(); j++)
			{
				TotalTranslationY[j] += IncrementTranslationY[j];
			};

			// Add Rotation Z.
			for (size_t k = 0; k < IncrementRotationZ.size(); k++)
			{
				//TotalRotationZ[k] += IncrementRotationZ[k];
			};
		}

		MyNodeLoadVectorNegativeTranslationX.push_back(TotalTranslationX);
		MyNodeLoadVectorNegativeTranslationY.push_back(TotalTranslationY);
		MyNodeLoadVectorNegativeRotationZ.push_back(TotalRotationZ);
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NonLinearStatic2D::SetBeamLoadVector()
{
	// *** POSITIVE ***
	// LoadStep
	vector <double> TotalTranslationX;
	vector <double> TotalTranslationY;
	vector <double> TotalRotationZ;

	// Loop through each Load Step.
	for (size_t i = 0; i < MyNonLinearFramePositive.size(); i++)
	{
		if (i == 0)
		{
			vector <int> DOF = MyNonLinearFramePositive[i].GetListOfDOFIndexes(2);

			// Find the Number of Beams.
			size_t NumOfDOF = DOF.size();

			// Declare the Matrix.
			vector <double> InitializeLoads;

			for (size_t j = 0; j < NumOfDOF; j++)
			{
				InitializeLoads.push_back(0.0);
			};

			// Set the Translation X.
			TotalTranslationX = InitializeLoads;
			
			// Set the Translation Y.
			TotalTranslationY = InitializeLoads;

			// Set the Rotation Z.
			TotalRotationZ = InitializeLoads;
		}
		else
		{
			// Set the Translation X.
			vector <double> IncrementTranslationX = MyNonLinearFramePositive[i].GetBeamGlobalLoadVector(ElasticFrame2D::TranslationX);

			// Set the Translation Y.
			vector <double> IncrementTranslationY = MyNonLinearFramePositive[i].GetBeamGlobalLoadVector(ElasticFrame2D::TranslationY);

			// Set the Rotation Z.
			vector <double> IncrementRotationZ = MyNonLinearFramePositive[i].GetBeamGlobalLoadVector(ElasticFrame2D::RotationZ);

			// Add Translation X.
			for (size_t h = 0; h < IncrementTranslationX.size(); h++)
			{
				TotalTranslationX[h] += IncrementTranslationX[h];
			};

			// Add Translation Y.
			for (size_t j = 0; j < IncrementTranslationY.size(); j++)
			{
				TotalTranslationY[j] += IncrementTranslationY[j];
			};

			// Add Rotation Z.
			for (size_t k = 0; k < IncrementRotationZ.size(); k++)
			{
				//TotalRotationZ[k] += IncrementRotationZ[k];
			};
		}

		MyBeamLoadVectorPositiveTranslationX.push_back(TotalTranslationX);
		MyBeamLoadVectorPositiveTranslationY.push_back(TotalTranslationY);
		MyBeamLoadVectorPositiveRotationZ.push_back(TotalRotationZ);
	};

	// LoadStep
	TotalTranslationX.clear();
	TotalTranslationY.clear();
	TotalRotationZ.clear();

	// *** NEGATIVE ***
	// Loop through each Load Step.
	for (size_t i = 0; i < MyNonLinearFrameNegative.size(); i++)
	{
		if (i == 0)
		{
			vector <int> DOF = MyNonLinearFramePositive[i].GetListOfDOFIndexes(2);

			// Find the Number of Beams.
			size_t NumOfDOF = DOF.size();

			// Declare the Matrix.
			vector <double> InitializeLoads;

			for (size_t j = 0; j < NumOfDOF; j++)
			{
				InitializeLoads.push_back(0.0);
			};


			// Set the Translation X.
			TotalTranslationX = InitializeLoads;

			// Set the Translation Y.
			TotalTranslationY = InitializeLoads;

			// Set the Rotation Z.
			TotalRotationZ = InitializeLoads;
		}
		else
		{
			// Set the Translation X.
			vector <double> IncrementTranslationX = MyNonLinearFrameNegative[i].GetBeamGlobalLoadVector(ElasticFrame2D::TranslationX);

			// Set the Translation Y.
			vector <double> IncrementTranslationY = MyNonLinearFrameNegative[i].GetBeamGlobalLoadVector(ElasticFrame2D::TranslationY);

			// Set the Rotation Z.
			vector <double> IncrementRotationZ = MyNonLinearFrameNegative[i].GetBeamGlobalLoadVector(ElasticFrame2D::RotationZ);

			// Add Translation X.
			for (size_t h = 0; h < IncrementTranslationX.size(); h++)
			{
				TotalTranslationX[h] += IncrementTranslationX[h];
			};

			// Add Translation Y.
			for (size_t j = 0; j < IncrementTranslationY.size(); j++)
			{
				TotalTranslationY[j] += IncrementTranslationY[j];
			};

			// Add Rotation Z.
			for (size_t k = 0; k < IncrementRotationZ.size(); k++)
			{
				//TotalRotationZ[k] += IncrementRotationZ[k];
			};
		}

		MyBeamLoadVectorNegativeTranslationX.push_back(TotalTranslationX);
		MyBeamLoadVectorNegativeTranslationY.push_back(TotalTranslationY);
		MyBeamLoadVectorNegativeRotationZ.push_back(TotalRotationZ);
	};
};

///<summary>Define the shape vector based off the characterisitics of the displacements.</summary>
///<remarks>description</remarks>
void NonLinearStatic2D::SetShapeVector()
{
	// *************** POSITIVE ********************
	// Translation X
	for (size_t i = 0; i < MyDisplacementVectorPositiveTranslationX.size(); i++)
	{
		// Store the Shape Vector.
		MyShapeVectorPositiveTranslationX.push_back(this -> GetNormalizeDisplacementVector(MyDisplacementVectorPositiveTranslationX[i]));
	};

	// Translation Y
	for (size_t j = 0; j < MyDisplacementVectorPositiveTranslationY.size(); j++)
	{
		// Store the Shape Vector.
		MyShapeVectorPositiveTranslationY.push_back(this-> GetNormalizeDisplacementVector(MyDisplacementVectorPositiveTranslationY[j]));
	};

	// Rotation Z
	for (size_t k = 0; k < MyDisplacementVectorPositiveRotationZ.size(); k++)
	{
		// Store the Shape Vector.
		MyShapeVectorPositiveRotationZ.push_back(this-> GetNormalizeDisplacementVector(MyDisplacementVectorPositiveRotationZ[k]));
	};

	// ************** NEGATIVE ******************
	// Translation X
	for (size_t i = 0; i < MyDisplacementVectorNegativeTranslationX.size(); i++)
	{
		// Store the Shape Vector.
		MyShapeVectorNegativeTranslationX.push_back(this -> GetNormalizeDisplacementVector(MyDisplacementVectorNegativeTranslationX[i]));
	};

	// Translation Y
	for (size_t j = 0; j < MyDisplacementVectorNegativeTranslationY.size(); j++)
	{
		// Store the Shape Vector.
		MyShapeVectorNegativeTranslationY.push_back(this -> GetNormalizeDisplacementVector(MyDisplacementVectorNegativeTranslationY[j]));
	};

	// Rotation Z
	for (size_t k = 0; k < MyDisplacementVectorNegativeRotationZ.size(); k++)
	{
		// Store the Shape Vector.
		MyShapeVectorNegativeRotationZ.push_back(this -> GetNormalizeDisplacementVector(MyDisplacementVectorNegativeRotationZ[k]));
	};
};


///<summary>Return the Normalized Displacement Vector.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> NonLinearStatic2D::GetNormalizeDisplacementVector(vector <double> DisplacementVector)
{
	// First we look for the Absolute Max
	/// TODO: Is it possible to find a more efficient method.
	double AbsMax = 0;

	// Loop through the Displacement Vector to look for absolute max.
	for (size_t i = 0; i < DisplacementVector.size(); i++)
	{
		if (std::abs(AbsMax) < std::abs(DisplacementVector[i]))
		{
			AbsMax = DisplacementVector[i];
		};
	};

	vector <double> ShapeVector;

	// Normalized Shape Vector.
	if (AbsMax != 0)
	{
		// If AbsMax is not equal to zero than we normalize it.
		for (size_t m = 0; m < DisplacementVector.size(); m++)
		{
			ShapeVector.push_back(DisplacementVector[m] / std::abs(AbsMax));
		};
	}
	else
	{
		// If not then all the values are zero and then just return the original.
		ShapeVector = DisplacementVector;
	}

	return ShapeVector;
};


///<summary>Generate the Shape Vector for the Translational X, Translational Y, and Rotational Z.</summary>
///<param name="DOF">Translation X, Translation Y, or Rotation Z.</param>
///<param name="LoadStep">The load step observing.</param>
///<param name="IsLoadingPositive">Are we looking at positive loading or negative loading.</param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> NonLinearStatic2D::GetShapeVector(int DOF, 
												  int LoadStep, 
												  bool IsLoadingPositive)
{
	// Initialize Shape Vector.
	vector <double> ShapeVector;

	if (IsLoadingPositive)
	{
		switch (DOF)
		{
			case NonLinearStatic2D::TranslationX:
				ShapeVector = MyShapeVectorPositiveTranslationX[LoadStep];
				break;

			case NonLinearStatic2D::TranslationY:
				ShapeVector = MyShapeVectorPositiveTranslationY[LoadStep];
				break;

			case NonLinearStatic2D::RotationZ:
				ShapeVector = MyShapeVectorPositiveRotationZ[LoadStep];
				break;
		};
	}
	else
	{
		switch (DOF)
		{
			case NonLinearStatic2D::TranslationX:
				ShapeVector = MyShapeVectorNegativeTranslationX[LoadStep];
				break;

			case NonLinearStatic2D::TranslationY:
				ShapeVector = MyShapeVectorNegativeTranslationY[LoadStep];
				break;

			case NonLinearStatic2D::RotationZ:
				ShapeVector = MyShapeVectorNegativeRotationZ[LoadStep];
				break;
		};
	};
	

	// Return the Shape Vector.
	return ShapeVector;
};

///<summary>Geerate the Mass Vector for each element Translational X, Translational Y, and Rotataional Z.</summary>
///<param name="DOF">Translation X, Translation Y, or Rotation Z.</param>
///<param name="LoadStep">The load step observing.</param>
///<param name="IsLoadingPositive">Are we looking at positive loading or negative loading.</param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> NonLinearStatic2D::GetMassVector(int DOF, 
												 int LoadStep, 
												 bool IsLoadingPositive)
{
	vector <double> MassVector;

	if (IsLoadingPositive)
	{
		switch (DOF)
		{
			case NonLinearStatic2D::TranslationX:
				MassVector = MyMassVectorPositiveTranslationX[LoadStep];
				break;

			case NonLinearStatic2D::TranslationY:
				MassVector = MyMassVectorPositiveTranslationY[LoadStep];
				break;

			case NonLinearStatic2D::RotationZ:
				MassVector = MyMassVectorPositiveRotationZ[LoadStep];
				break;
		};
	}
	else
	{
		switch (DOF)
		{
			case NonLinearStatic2D::TranslationX:
				MassVector = MyMassVectorNegativeTranslationX[LoadStep];
				break;

			case NonLinearStatic2D::TranslationY:
				MassVector = MyMassVectorNegativeTranslationY[LoadStep];
				break;

			case NonLinearStatic2D::RotationZ:
				MassVector = MyMassVectorNegativeRotationZ[LoadStep];
				break;
		};
	};

	// Return the Mass Vector.
	return MassVector;
};

///<summary>Get the total mass for the load step<summary>
///<param name="DOF">Translation X, Translation Y, or Rotation Z.</param>
///<param name="LoadStep">The load step observing.</param>
///<param name="IsLoadingPositive">Are we looking at positive loading or negative loading.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double NonLinearStatic2D::GetTotalMass(int DOF, 
									   int LoadStep, 
									   bool IsLoadingPositive)
{
	double SumOfMass = 0.0;

	vector <double> MassVector = this -> GetMassVector(DOF, LoadStep, IsLoadingPositive);

	for (int i = 0; i < MassVector.size(); i++)
	{
		SumOfMass += MassVector[i];
	};

	return SumOfMass;
};

///<summary>Return the Load at Each Beam Node.</summary>
///<param name="DOF">Translation X, Translation Y, or Rotation Z.</param>
///<param name="LoadStep">The load step observing.</param>
///<param name="IsLoadingPositive">Are we looking at positive loading or negative loading.</param>
///<remarks>description</remarks>
vector <double> NonLinearStatic2D::GetBeamLoadVector(int DOF, 
												     int LoadStep, 
													 bool IsLoadingPositive)
{
	vector <double> LoadVector;

	if (IsLoadingPositive)
	{
		// Add the Desire Load Vector together.
		switch(DOF)
		{
			case NonLinearStatic2D::TranslationX:
				LoadVector = MyBeamLoadVectorPositiveTranslationX[LoadStep];
				break;
			case NonLinearStatic2D::TranslationY:
				LoadVector = MyBeamLoadVectorPositiveTranslationY[LoadStep];
				break;
			case NonLinearStatic2D::RotationZ:
				LoadVector = MyBeamLoadVectorPositiveRotationZ[LoadStep];
				break;
		};
	}
	else
	{
		// Add the Desire Load Vector together.
		switch(DOF)
		{
			case NonLinearStatic2D::TranslationX:
				LoadVector = MyBeamLoadVectorNegativeTranslationX[LoadStep];
				break;
			case NonLinearStatic2D::TranslationY:
				LoadVector = MyBeamLoadVectorNegativeTranslationY[LoadStep];
				break;
			case NonLinearStatic2D::RotationZ:
				LoadVector = MyBeamLoadVectorNegativeRotationZ[LoadStep];
				break;
		};
	};

	return LoadVector;
};

///<summary>Return the Load at each Node.</summary>
///<param name="DOF">Translation X, Translation Y, or Rotation Z.</param>
///<param name="LoadStep">The load step observing.</param>
///<param name="IsLoadingPositive">Are we looking at positive loading or negative loading.</param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> NonLinearStatic2D::GetNodeLoadVector(int DOF, 
													 int LoadStep, 
													 bool IsLoadingPositive)
{
	vector <double> LoadVector;

	if (IsLoadingPositive)
	{
		// Add the desire load vector together;
		switch(DOF)
		{
			case NonLinearStatic2D::TranslationX:
				LoadVector = MyBeamLoadVectorPositiveTranslationX[LoadStep];
				break;
			case NonLinearStatic2D::TranslationY:
				LoadVector = MyBeamLoadVectorPositiveTranslationY[LoadStep];
				break;
			case NonLinearStatic2D::RotationZ:
				LoadVector = MyBeamLoadVectorPositiveRotationZ[LoadStep];
				break;
		};
	}
	else
	{
		// Add the desire load vector together;
		switch(DOF)
		{
			case NonLinearStatic2D::TranslationX:
				LoadVector = MyBeamLoadVectorNegativeTranslationX[LoadStep];
				break;
			case NonLinearStatic2D::TranslationY:
				LoadVector = MyBeamLoadVectorNegativeTranslationY[LoadStep];
				break;
			case NonLinearStatic2D::RotationZ:
				LoadVector = MyBeamLoadVectorNegativeRotationZ[LoadStep];
				break;
		};
	};

	return LoadVector;
};

///<summary>Return a vector of the Mass Factor.</summary>
///<param name="DOF">Translation X, Translation Y, or Rotation Z.</param>
///<param name="IsLoadingPositive">Are we looking at positive loading or negative loading.</param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> NonLinearStatic2D::GetMassFactorVector(int DOF, 
													   bool IsLoadingPositive)
{
	vector <double> MassFactor;

	for (size_t i = 0; i < MyNonLinearFramePositive.size(); i++)
	{
		double Numerator = 0.0;
		double Denominator = 0.0;

		// Get the Mass Vector for the Load Step.
		vector <double> MassVector = this -> GetMassVector(DOF, i, IsLoadingPositive);
		vector <double> ShapeVector = this -> GetShapeVector(DOF, i, IsLoadingPositive);

		// Loop through the Sequience.
		for (size_t j = 0; j < ShapeVector.size(); j++)
		{
			// Summation of the Equivalent Mass = Mass * Phi^2.
			Numerator += MassVector[j] * pow(ShapeVector[j], 2);
	
			// Summatin of the Total Mass
			Denominator += MassVector[j];
		};

		// Add to the Mass Factor.
		MassFactor.push_back(Numerator / Denominator);
	};

	return MassFactor;
};

///<summary>Return a vector of the Load Factor.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
vector <double> NonLinearStatic2D::GetLoadFactorVector(int DOF, 
													   bool IsLoadingPositive)
{
	vector <double> LoadFactor;

	for (size_t i = 0; i < MyNonLinearFramePositive.size(); i++)
	{
		double Numerator = 0.0;
		double Denominator = 0.0;

		// Get the Load Vector for the Load Step.
		vector <double> BeamLoadVector = this -> GetBeamLoadVector(DOF, i, IsLoadingPositive);
		vector <double> ShapeVector = this -> GetShapeVector(DOF, i, IsLoadingPositive);

		// Loop through the Sequience.
		for (size_t j = 0; j < ShapeVector.size(); j++)
		{
			// Summation of the Equivalent Load = Load * Phi^2.
			Numerator += BeamLoadVector[j] * ShapeVector[j];
	
			// Summatin of the Total Load
			Denominator += BeamLoadVector[j];
		};

		// Add to the Load Factor.
		LoadFactor.push_back(Numerator / Denominator);
	};

	return LoadFactor;
};

///<summary>Return a vector of the Total Load throughout the Loading History.</summary>
///<param name="DOF">Degrees-of-freedom direction that the function is looking for.</param>
///<param name="IsLoadingPositive">Determine if the loading is positive.</param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> NonLinearStatic2D::GetTotalHistoryLoad(int DOF, 
													   bool IsLoadingPositive)
{
	vector <double> TotalLoad;

	// Get the Total Reference Load.
	double ReferenceTotalLoad = this -> GetTotalReferenceLoad(DOF);

	if (IsLoadingPositive)
	{
		// Add the first total Load.
		TotalLoad.push_back(ReferenceTotalLoad * MyFinalLoadRatiosPositive[0]);

		for (size_t i = 1; i < MyFinalLoadRatiosPositive.size(); i++)
		{
			// Add the previous load with the reference load.
			TotalLoad.push_back(TotalLoad.back() + ReferenceTotalLoad * MyFinalLoadRatiosPositive[i]);
		};
	}
	else
	{
		// Add the first total Load.
		TotalLoad.push_back(ReferenceTotalLoad * MyFinalLoadRatiosNegative[0]);

		for (size_t i = 1; i < MyFinalLoadRatiosPositive.size(); i++)
		{
			// Add the previous load with the reference load.
			TotalLoad.push_back(TotalLoad.back() + ReferenceTotalLoad * MyFinalLoadRatiosNegative[i]);
		};
	};
	
	return TotalLoad;
};

///<summary></summary>
///<param name="LoadStep"></param>
///<param name="DOF"></param>
///<param name="IsLoadingPositive"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double NonLinearStatic2D::GetTotalLoad(int LoadStep, 
									   int DOF, 
									   bool IsLoadingPositive)
{
	// Declare the Nodal Variable
	double SumOfNodeLoads = 0.0;
	
	// Declare the Total Summation Variable.
	double SumOfBeamLoads = 0.0;
	
	// Check if Load Step exist first.
	if (LoadStep < MyNonLinearFramePositive.size())
	{
		vector <double> NodeLoads = this -> GetNodeLoadVector(DOF, LoadStep, IsLoadingPositive);
		vector <double> BeamLoads = this -> GetBeamLoadVector(DOF, LoadStep, IsLoadingPositive);

		for (size_t i = 0; i < NodeLoads.size(); i++)
		{
			// Add all the Node Loads.
			SumOfNodeLoads += NodeLoads[i];
		};

		for (size_t j = 0; j < BeamLoads.size(); j++)
		{
			SumOfBeamLoads += BeamLoads[j];
		};
	}

	// Return Load
	return SumOfNodeLoads + SumOfBeamLoads;
};

///<summary>Find the total load converted into forces for the DOF.</summary>
///<param name="DOF">Degree of freedom that is addressed.</param>
///<returns>description</returns>
///<remarks>description</remarks>
double NonLinearStatic2D::GetTotalReferenceLoad(int DOF)
{
	// Set the Reference Frame.
	ElasticFrame2D ReferenceFrame = MyBaseFrame;

	// Set the Loads.
	this -> SetIncrementLoad(ReferenceFrame, 1.0);

	// Run the Static Analysis.
	ReferenceFrame.RunStaticAnalysis();

	vector <double> LoadVector = ReferenceFrame.GetBeamGlobalLoadVector(DOF);

	double TotalLoad = 0;

	for (size_t i = 0; i < LoadVector.size(); i++)
	{
		TotalLoad += LoadVector[i];
	};

	return TotalLoad;
};

///<summary>Scan through the nodes at</summary>
///<param name="NodeDOF"></param>
///<param name="LoadIncrement"></param>
///<param name="IsPositiveLoading"></param>
///<returns>description</returns>
///<remarks>description</remarks>
int NonLinearStatic2D::GetNodeIDWithLargestDeformation(int NodeDOF,
													   int LoadIncrement,
													   bool IsPositiveLoading)
{
	int NodeID = -1;

	// Return the base location global displacements.
	vector <mat> BaseLocation = MyBaseFrame.GetNodesGlobalDisplacement();

	// Find the increment location global displacements.
	vector <mat> IncrementLocation;

	if (IsPositiveLoading) 
	{
		IncrementLocation = MyNonLinearFramePositive[LoadIncrement].GetNodesGlobalDisplacement();	
	}
	else
	{
		IncrementLocation = MyNonLinearFrameNegative[LoadIncrement].GetNodesGlobalDisplacement();
	};

	vector <double> Displacements;

	// Loop through to find the difference at each node.
	for (size_t i = 0; i < BaseLocation.size(); i++)
	{
		double Initial = BaseLocation[i](NodeDOF-1, 0);
		double Increment = IncrementLocation[i](NodeDOF-1, 0);

		Displacements.push_back(Initial - Increment);
	};

	int Index = 0;
	double MaxDisplacements = abs(Displacements[0]);

	for (size_t j = 1; j < Displacements.size(); j++)
	{
		// Compare the increment.
		if (abs(Displacements[j]) > MaxDisplacements)
		{
			Index = j;
			MaxDisplacements = abs(Displacements[j]);
		}
	};

	// Return the Node Vector.
	vector <Node2D> Nodes = MyBaseFrame.GetNodes();

	return Nodes[Index].GetID();
};

///<summary>Return the Resistance Function for the Node ID.</summary>
///<param name="NodeID">ID of the Node.</param>
///<param name="NodeDOF">Desired Degree of Freedom of the Node to Track.</param>
///<param name="LoadDOF">Desired DOF direction of the Node.</param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <vector<double>> NonLinearStatic2D::GetResistanceFunction(int NodeID, 
																 int NodeDOF, 
																 int LoadDOF, 
																 bool IsPositiveLoading)
{
	vector <vector<double>> ResistanceFunction;

	// Return the Load History.
	vector <double> Load = this -> GetTotalHistoryLoad(LoadDOF, IsPositiveLoading);

	// Return the Displacement of the Node Load.
	vector <Node2D> NodeHistory = this -> GetNodeLoadHistory(NodeID, IsPositiveLoading);

	// Loop.
	for (size_t i = 0; i < NodeHistory.size(); i++)
	{
		// Set 0 Load;
		vector <double> RF;

		// Set the Displacement.
		switch (NodeDOF)
		{
			case ElasticFrame2D::TranslationX:
				RF.push_back(NodeHistory[i].GetX());
				break;

			case ElasticFrame2D::TranslationY:
				RF.push_back(NodeHistory[i].GetY());
				break;

			case ElasticFrame2D::RotationZ:
				RF.push_back(NodeHistory[i].GetRotationZ());
				break;
		};

		// Set the Load.
		RF.push_back(Load[i]);

		// Store in Vector;
		ResistanceFunction.push_back(RF);
	};

	return ResistanceFunction;
};

///<summary>Return the Load-Mass factor.</summary>
///<param name="NodeID"></param>
///<param name="NodeDOF"></param>
///<param name="LoadDOF"></param>
///<param name="IsPositiveLoading"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <vector<double>> NonLinearStatic2D::GetLoadMassFactor(int NodeID,
															 int NodeDOF,
															 int LoadDOF,
															 bool IsPositiveLoading)
{
	vector <vector<double>> LoadMassFactor;

	// Return the Displacement of the Node Load.
	vector <Node2D> NodeHistory = this -> GetNodeLoadHistory(NodeID, IsPositiveLoading);

	vector <double> MassFactor = this -> GetMassFactorVector(NodeDOF, IsPositiveLoading);
	vector <double> LoadFactor = this -> GetLoadFactorVector(LoadDOF, IsPositiveLoading);

	// Loop.
	for (size_t i = 0; i < NodeHistory.size(); i++)
	{
		// Set 0 Load;
		vector <double> LM;

		// Set the Displacement.
		switch (NodeDOF)
		{
			case ElasticFrame2D::TranslationX:
				LM.push_back(NodeHistory[i].GetX());
				break;

			case ElasticFrame2D::TranslationY:
				LM.push_back(NodeHistory[i].GetY());
				break;

			case ElasticFrame2D::RotationZ:
				LM.push_back(NodeHistory[i].GetRotationZ());
				break;
		};

		// Set the Load.
		if (i == 0)
		{
			LM.push_back(abs(MassFactor[1] / LoadFactor[1]));
		}
		else
		{
			LM.push_back(abs(MassFactor[i] / LoadFactor[i]));
		};
		
		// Store in Vector;
		LoadMassFactor.push_back(LM);
	};

	return LoadMassFactor;
};


#pragma endregion 

///<summary>Set the gravity constant of the section.</summary>
///<param name="Gravity">Gravity constant.</param>
///<returns>description</returns>
///<remarks>description</remarks>
void NonLinearStatic2D::SetGravity(double Gravity)
{
	MyGravity = Gravity;
};

///<summary>Return the gravity of the section.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
double NonLinearStatic2D::GetGravity()
{
	return MyGravity;
};