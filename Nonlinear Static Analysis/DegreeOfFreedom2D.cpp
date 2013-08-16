#include "StdAfx.h"
#include "DegreeOfFreedom2D.h"

///<summary></summary>
///<param name="ID"></param>
///<remarks>description</remarks>
void DegreeOfFreedom2D::Initialize(int ID)
{
	MyID = ID;
};

///<summary></summary>
///<param name="DOF"></param>
///<remarks>description</remarks>
void DegreeOfFreedom2D::Clone(const DegreeOfFreedom2D &DOF)
{
	MyID = DOF.MyID;

	MyVectorBeams = DOF.MyVectorBeams;
};

///<summary></summary>
///<param name="ID"></param>
///<remarks>description</remarks>
DegreeOfFreedom2D::DegreeOfFreedom2D(int ID)
{
	this -> Initialize(ID);
};

///<summary></summary>
///<remarks>description</remarks>
DegreeOfFreedom2D::DegreeOfFreedom2D(void)
{
	this -> Initialize();
};

///<summary></summary>
///<remarks>description</remarks>
DegreeOfFreedom2D::~DegreeOfFreedom2D(void)
{
	
};

///<summary></summary>
///<param name="DOF"></param>
///<remarks>description</remarks>
DegreeOfFreedom2D& DegreeOfFreedom2D::operator= (const DegreeOfFreedom2D &DOF)
{
	this -> Clone(DOF);

	return *this;
};

///<summary></summary>
///<param name="DOF"></param>
///<remarks>description</remarks>
DegreeOfFreedom2D::DegreeOfFreedom2D(const DegreeOfFreedom2D &DOF)
{
	this -> Clone(DOF);
};

///<summary></summary>
///<param name="Beam"></param>
///<remarks>description</remarks>
void DegreeOfFreedom2D::AddBeam(Beam2D *Beam)
{
	MyVectorBeams.push_back(Beam);
};

///<summary></summary>
///<param name="Beam"></param>
///<remarks>description</remarks>
void DegreeOfFreedom2D::AddBeam(vector <Beam2D*> Beam)
{
	for (size_t i; i < Beam.size(); i++)
	{
		this -> AddBeam(Beam[i]);
	};
};
