// $Id: NonLinearAnalyticConditionalGaussian3D.cpp 5823 2005-10-27 13:43:02Z TDeLaet $
// Copyright (C) 2006  Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "bayesian_filtering/nonlinearanalyticconditionalgaussian3D.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng
// libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
using namespace MatrixWrapper;


NonLinearAnalyticConditionalGaussian3D::NonLinearAnalyticConditionalGaussian3D(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE)
{}

NonLinearAnalyticConditionalGaussian3D::~NonLinearAnalyticConditionalGaussian3D(){}

ColumnVector NonLinearAnalyticConditionalGaussian3D::ExpectedValueGet() const
{
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector input  = ConditionalArgumentGet(1); // Transformation

    // Construct homogeneous transformation matrix
    Matrix T_homogeneous(4,4);
    T_homogeneous=0;

    // Rotation
    T_homogeneous(1,1)=1.0;
    T_homogeneous(2,2)=1.0;
    T_homogeneous(3,3)=1.0;

    // Translation
    T_homogeneous(1,4)=input(1); // delta_x
    T_homogeneous(2,4)=input(2); // delta_y
    T_homogeneous(3,4)=input(3); // delta_z
    T_homogeneous(4,4)=1.0;

    // Construct homogeneous state vector
    ColumnVector state_homogeneous(4);
    state_homogeneous=0;
    state_homogeneous(1)=state(1);     state_homogeneous(2)=state(2);    state_homogeneous(3)=state(3);    state_homogeneous(4)=1.0;

    // Apply transformation
    state_homogeneous=T_homogeneous * state_homogeneous;

    // Convert back to state coordinates
    state(1)=state_homogeneous(1);   state(2)=state_homogeneous(2);  state(3)=state_homogeneous(3);
    return state + AdditiveNoiseMuGet();
}

Matrix NonLinearAnalyticConditionalGaussian3D::dfGet(unsigned int i) const
{
    if (i==0)//derivative to the first conditional argument (x)
    {
        ColumnVector state = ConditionalArgumentGet(0);
        ColumnVector vel = ConditionalArgumentGet(1);
        Matrix df(3,3);
        df(1,1)=1;
        df(1,2)=0;
        df(1,3)=-vel(1)*sin(state(3));
        df(2,1)=0;
        df(2,2)=1;
        df(2,3)=vel(1)*cos(state(3));
        df(3,1)=0;
        df(3,2)=0;
        df(3,3)=1;


        return df;
    }
    else
    {
        if (i >= NumConditionalArgumentsGet())
        {
            cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
            exit(-BFL_ERRMISUSE);
        }
        else
        {
            cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
            exit(-BFL_ERRMISUSE);
        }
    }
}

}//namespace BFL


