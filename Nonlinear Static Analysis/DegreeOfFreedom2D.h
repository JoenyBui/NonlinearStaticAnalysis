#pragma once

#ifndef DegreeOfFreedom2D_H
#define DegreeOfFreedom2D_H

#include "stdafx.h"
#include "Beam2D.h"

class Beam2D;

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
class DegreeOfFreedom2D
{
	private:
		int MyID;
		
		vector <Beam2D*> MyVectorBeams;

		void Initialize(int ID = 0);

		void Clone(const DegreeOfFreedom2D &DOF);
	public:
		DegreeOfFreedom2D(int ID);

		DegreeOfFreedom2D(void);

		~DegreeOfFreedom2D(void);

		DegreeOfFreedom2D& operator= (const DegreeOfFreedom2D &DOF);

		DegreeOfFreedom2D(const DegreeOfFreedom2D& DOF);

		void AddBeam(Beam2D* Beam);

		void AddBeam(vector <Beam2D*> Beam);
};

#endif