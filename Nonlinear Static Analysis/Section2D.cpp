#include "StdAfx.h"
#include "Section2D.h"

///<summary>Constructor for the Bi-Linear Model of the material.</summary>
///<param name="ID">ID of the Material Model.</param>
///<param name="GrossArea">Gross Cross Sectional Area of the Material Section.</param>
///<param name="GrossMomentOfInertia">Gross Moment of Inertia of the strong axis of the Material Section.</param>
///<param name="NeutralMoment">Moment at zero curvature.</param>
///<param name="PlasticMomentPositive">Plastic Moment on the Positive Side of the Material Section.</param>
///<param name="YieldCurvaturePositive">Yielding Curvature on the Positive Side of the Material Section.</param>
///<param name="UltimateCurvaturePositive">Ultiamte Moment Curvature on the Positive Side of the Material Section.</param>
///<param name="PlasticMomentNegative">Plastic Moment on the Negative Side of the Material Section.</param>
///<param name="YieldCurvatureNegative">Yielding Curvatuer on the Negative Side of the Material Section.</param>
///<param name="UltimateCurvatureNegative">Ultimate Moment Curvature on the Negative Side of the Material Section.</param>
///<remarks>All Default are to 0.0.</remarks>
void Section2D::Initialize(int ID = -1, 
						   double GrossArea = 0.0,
						   double GrossMomentOfInertia = 0.0,
						   double NeutralMoment = 0.0,
						   double PlasticMomentPositive = 0.0, 
						   double YieldCurvaturePositive = 0.0, 
						   double UltimateCurvaturePositive = 0.0, 
						   double PlasticMomentNegative = 0.0, 
						   double YieldCurvatureNegative = 0.0, 
						   double UltimateCurvatureNegative = 0.0,
						   double Density = 1.0)
{
	// Set the Criteria.
	MyID = ID;

	MyGrossArea = GrossArea;
	MyGrossMomentOfInertia = GrossMomentOfInertia;

	MyNeutralMoment = NeutralMoment;

	MyPlasticMomentPositive = PlasticMomentPositive;
	MyYieldCurvaturePositive = YieldCurvaturePositive;
	MyUltimateCurvaturePositive = UltimateCurvaturePositive;
	
	MyPlasticMomentNegative = PlasticMomentNegative;
	MyYieldCurvatureNegative = YieldCurvatureNegative;
	MyUltimateCurvatureNegative = UltimateCurvatureNegative;

	// Set the Elastic Modulus.
	MyElasticModulusPositive = ((MyPlasticMomentPositive - MyNeutralMoment) / (MyYieldCurvaturePositive - 0)) / MyGrossMomentOfInertia;
	MyElasticModulusNegative = ((MyNeutralMoment - MyPlasticMomentNegative) / (0 - MyYieldCurvatureNegative)) / MyGrossMomentOfInertia;

	// Set the Density of the Material.
	MyMassDensity = Density;
};

///<summary>Clone the corresponding the Section2D.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Section2D::Clone(const Section2D &Section)
{
	// Section ID.
	MyID = Section.MyID;
		
	// Geometry.
	MyGrossArea = Section.MyGrossArea;
	MyGrossMomentOfInertia = Section.MyGrossMomentOfInertia;

	MyMassDensity = Section.MyMassDensity;

	MyNeutralMoment = Section.MyNeutralMoment;

	// Positive M-PHI Diagram.
	MyPlasticMomentPositive = Section.MyPlasticMomentPositive;
	MyYieldCurvaturePositive = Section.MyYieldCurvaturePositive;
	MyUltimateCurvaturePositive = Section.MyUltimateCurvaturePositive;
	MyElasticModulusPositive = Section.MyElasticModulusPositive;

	// Negative M-PHI Diagram.
	MyPlasticMomentNegative = Section.MyPlasticMomentNegative;
	MyYieldCurvatureNegative = Section.MyYieldCurvatureNegative;
	MyUltimateCurvatureNegative = Section.MyUltimateCurvatureNegative;
	MyElasticModulusNegative = Section.MyElasticModulusNegative;
};

///<summary>Constructor for the Bi-Linear Model of the material.</summary>
///<param name="ID">ID of the Material Model.</param>
///<param name="GrossArea">Gross Cross Sectional Area of the Material Section.</param>
///<param name="GrossMomentOfInertia">Gross Moment of Inertia of the strong axis of the Material Section.</param>
///<param name="PlasticMomentPositive">Plastic Moment on the Positive Side of the Material Section.</param>
///<param name="YieldCurvaturePositive">Yielding Curvature on the Positive Side of the Material Section.</param>
///<param name="UltimateCurvaturePositive">Ultimate Moment Curvature on the Positive Side of the Material Section.</param>
///<param name="PlasticMomentNegative">Plastic Moment on the Negative Side of the Material Section.</param>
///<param name="YieldCurvatureNegative">Yielding Curvatuer on the Negative Side of the Material Section.</param>
///<param name="UltimateCurvatureNegative">Ultimate Moment Curvature on the Negative Side of the Material Section.</param>
///<remarks>description</remarks>
Section2D::Section2D(int ID, 
					 double GrossArea, 
					 double GrossMomentOfInertia, 
					 double PlasticMomentPositive, 
					 double YieldCurvaturePositive, 
					 double UltimateCurvaturePositive, 
					 double PlasticMomentNegative, 
					 double YieldCurvatureNegative, 
					 double UltimateCurvatureNegative,
					 double Density)
{
	// Find the Neutral Moment.
	double EI = (PlasticMomentPositive - PlasticMomentNegative) / (YieldCurvaturePositive - YieldCurvatureNegative);

	double NeutralMoment = EI * YieldCurvatureNegative + PlasticMomentNegative;

	// Initialize the Section.
	this -> Initialize(ID, 
					   GrossArea, 
					   GrossMomentOfInertia, 
					   NeutralMoment,
					   PlasticMomentPositive, 
					   YieldCurvaturePositive, 
					   UltimateCurvaturePositive, 
					   PlasticMomentNegative, 
					   YieldCurvatureNegative, 
					   UltimateCurvatureNegative,
					   Density);
};

///<summary>Constructor for the Bi-Linear Model of the material.</summary>
///<param name="ID">ID of the Material Model.</param>
///<param name="GrossArea">Gross Cross Sectional Area of the Material Section.</param>
///<param name="GrossMomentOfInertia">Gross Moment of Inertia of the strong axis of the Material Section.</param>
///<param name="GrossElasticModulus">Gross Elastic Modulus of the Material Section.</param>
///<param name="Curvature">Yielding Curvature Positive, Ultimate Curvature Positive, Yield Curvature Negative, Ultimate Curvature Negative, Initial Cuvature.</param>
Section2D::Section2D(int ID, 
					 double GrossArea, 
					 double GrossMomentOfInertia, 
					 double GrossElasticModulus, 
					 vector <double> Curvature,
					 double Density)
{
	double YieldCurvaturePositive = Curvature[0]; 
	double UltimateCurvaturePositive = Curvature[1];
	double YieldCurvatureNegative = Curvature[2];
	double UltimateCurvatureNegative = Curvature[3];
	double InitialCurvature = Curvature[4];

	// Retreive the Load in terms of the Moment Curvature Variables.
	double EI = GrossMomentOfInertia * GrossElasticModulus;

	// Plastic Moment.
	double Mp_Positive = EI * (YieldCurvaturePositive - InitialCurvature);
	double Mp_Negative = EI * (YieldCurvatureNegative - InitialCurvature);
	double M_Neutral = abs(EI * YieldCurvatureNegative) + Mp_Negative;

	// Initialize the Section.
	this -> Initialize(ID, 
					   GrossArea, 
					   GrossMomentOfInertia, 
					   M_Neutral,
					   Mp_Positive, 
					   YieldCurvaturePositive, 
					   UltimateCurvaturePositive, 
					   Mp_Negative, 
					   YieldCurvatureNegative, 
					   UltimateCurvatureNegative,
					   Density);

	// Establish the Modulus.
	MyElasticModulusPositive =  GrossElasticModulus;
	MyElasticModulusNegative =  GrossElasticModulus;
};

///<summary>Define the section bi-linear properties from GirderSection2D.  The 
/// model will use the "true" moment capacity and not the "applied" moment capacity.
/// This means all the coordinates are shifted relative to the initial applied moment.
/// Also, the stiffness of the models are averaged out between the yielding curvature.</summary>
///<param name="ID"></param>
///<param name="GrossArea"></param>
///<param name="GrossMomentOfInertia"></param>
///<param name="PositiveCoordinates"></param>
///<param name="NegativeCoordinates"></param>
///<param name="Density"></param>
///<remarks>description</remarks>
Section2D::Section2D(int ID, 
					 double GrossArea, 
					 double GrossMomentOfInertia, 
					 double InitialMoment,
					 std::vector <vector<double>> PositiveCoordinates, 
					 std::vector <vector<double>> NegativeCoordinates, 
					 double Density)
{
	// Find the Positive Ultimate Moment.
	double PositiveUltimateMoment = (PositiveCoordinates[2][1]);

	// Find the Positive Stiffness = EI.
	double PositiveStiffness = (PositiveCoordinates[1][1] - PositiveCoordinates[0][1])	/ PositiveCoordinates[1][0];

	// Find the Positive Yield Curvature.
	double PositiveYieldCurvature = (PositiveCoordinates[2][1] - PositiveCoordinates[0][1]) / PositiveStiffness;

	// Find the Positive Ultimate Curvature.
	double PositiveUltimateCurvature = (PositiveCoordinates[2][0]);

	// Find the Negative Ultimate Moment.
	double NegativeUltimateMoment = (NegativeCoordinates[2][1]);

	// Find the Negative Stiffness = EI.
	double NegativeStiffness = (NegativeCoordinates[1][1] - NegativeCoordinates[0][1])	/ NegativeCoordinates[1][0];

	// Find the Negative Yield Curvature.
	double NegativeYieldCurvature = (NegativeCoordinates[2][1] - NegativeCoordinates[0][1]) / NegativeStiffness;

	// Find the Negative Ultimate Curvature.
	double NegativeUltimateCurvature = NegativeCoordinates[2][0];

	// Set the Neutral Moment (could be taken from either positive or negative coordinate.
	double NeutralMoment = NegativeUltimateMoment + (PositiveUltimateMoment - NegativeUltimateMoment) * (0 - NegativeYieldCurvature) 
																										/ (PositiveYieldCurvature - NegativeYieldCurvature);
	//double NeutralMoment = (PositiveCoordinates[0][1]);

	// Initialize the class.
	this -> Initialize(ID, 
					   GrossArea,
					   GrossMomentOfInertia,
					   NeutralMoment + InitialMoment,
					   PositiveUltimateMoment + InitialMoment,
					   PositiveYieldCurvature,
					   PositiveUltimateCurvature,
					   NegativeUltimateMoment + InitialMoment,
					   NegativeYieldCurvature,
					   NegativeUltimateCurvature,
					   Density);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
Section2D::Section2D(void)
{
	this -> Initialize();
}

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
Section2D::~Section2D(void)
{
}

void Section2D::Print()
{
	cout << "Negative Ultimate Moment Capacity";
};

///<summary>Return the copy of the section.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
Section2D& Section2D::operator= (const Section2D &Section)
{
	this -> Clone(Section);

	// Return a Pointer to this Section.
	return *this;
};

///<summary>Copy a section.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
Section2D::Section2D(const Section2D &Section)
{
	this -> Clone(Section);
};

///<summary>Return the Section ID.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
int Section2D::GetID()
{
	return MyID;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetArea()
{
	return MyGrossArea;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetMomentOfInertia()
{
	return MyGrossMomentOfInertia;
};

///<summary>Return the average elastic modulus of the section.  The section uses the average of the positive/negative modulus.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetElasticModulus()
{
	return (abs(MyElasticModulusPositive) + abs(MyElasticModulusNegative)) / 2.0;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetElasticModulus(double Curvature)
{
	double ElasticModulus = 0.0;

	if (Curvature > 0.0)
	{
		if (Curvature >= MyYieldCurvaturePositive)
		{
			// The section is beyond the yield therefore it is plastic.
			ElasticModulus = 0.0;
		}
		else
		{
			// The section is positive.
			ElasticModulus = MyElasticModulusPositive;
		};
		
	}
	else
	{
		if (Curvature <= MyYieldCurvatureNegative)
		{
			// The section is beyond the yield therefore it is plastic.
			ElasticModulus = 0.0;
		}
		else
		{
			// The section is negative.
			ElasticModulus = MyElasticModulusPositive;
		};
	};

	return ElasticModulus;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetStress(double Strain)
{
	return 0;
};

///<summary>Return the Strain in the section given the Stress.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetStrain(double Stress)
{
	return 0;
};

///<summary>Return the Positive Plastic Moment Capacity of the section.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetPositivePlasticMoment()
{
	return MyPlasticMomentPositive;
};

///<summary>Return the Negative Plastic Moment Capacity of the section.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetNegativePlasticMoment()
{
	return MyPlasticMomentNegative;
};

///<summary>Return the mass density of the section.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetDensity()
{
	return MyMassDensity;
};

///<summary>Return the moment curvature vector of the section.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
vector <vector<double>> Section2D::GetMomentCurvatureVector()
{
	vector <vector<double>> MomentCurvature;

	vector <double> NegativeUltimate;
	NegativeUltimate.push_back(MyUltimateCurvatureNegative);
	NegativeUltimate.push_back(MyPlasticMomentNegative);

	vector <double> NegativeYield;
	NegativeYield.push_back(MyYieldCurvatureNegative);
	NegativeYield.push_back(MyPlasticMomentNegative);

	vector <double> NoCurvature;
	NoCurvature.push_back(0.0);
	NoCurvature.push_back(MyNeutralMoment);

	vector <double> PositiveYield;
	PositiveYield.push_back(MyYieldCurvaturePositive);
	PositiveYield.push_back(MyPlasticMomentPositive);

	vector <double> PositiveUltimate;
	PositiveUltimate.push_back(MyUltimateCurvaturePositive);
	PositiveUltimate.push_back(MyPlasticMomentPositive);

	MomentCurvature.push_back(NegativeUltimate);
	MomentCurvature.push_back(NegativeYield);
	MomentCurvature.push_back(NoCurvature);
	MomentCurvature.push_back(PositiveYield);
	MomentCurvature.push_back(PositiveUltimate);

	return MomentCurvature;
};

///<summary>Return the Mass per Unit Length.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
double Section2D::GetMassPerUnitLength()
{
	return MyGrossArea * MyMassDensity;
};