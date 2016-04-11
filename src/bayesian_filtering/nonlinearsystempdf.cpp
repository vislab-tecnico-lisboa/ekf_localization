// $Id: nonlinearanalyticconditionalgaussianmobile.cpp 5823 2005-10-27 13:43:02Z TDeLaet $
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

#include "bayesian_filtering/nonlinearsystempdf.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng libraries

#define SYSMODEL_NUMCONDARGUMENTS_MOBILE 2
#define SYSMODEL_DIMENSION_MOBILE        3

namespace BFL
{
using namespace MatrixWrapper;

NonlinearSystemPdf::NonlinearSystemPdf(const Gaussian& additiveNoise)
    : ConditionalPdf<ColumnVector,ColumnVector>(SYSMODEL_DIMENSION_MOBILE,SYSMODEL_NUMCONDARGUMENTS_MOBILE)
{
    _additiveNoise = additiveNoise;
}

NonlinearSystemPdf::~NonlinearSystemPdf(){}

bool NonlinearSystemPdf::SampleFrom(Sample<ColumnVector>& one_sample, int method, void * args) const
{
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector input  = ConditionalArgumentGet(1); // Transformation


    // Construct homogeneous transformation matrix
    Matrix T_homogeneous(4,4);
    //T_homogeneous=0;

    for(unsigned int row=1; row<=4; ++row)
    {
        for(unsigned int col=1; col<=4; ++col)
        {
            unsigned int index=col+(row-1)*4;
            T_homogeneous(row,col)=input(index);
        }
    }

    // Construct homogeneous state vector
    ColumnVector state_homogeneous(4);
    state_homogeneous=0;
    state_homogeneous(1)=state(1);     state_homogeneous(2)=state(2);    state_homogeneous(3)=state(3);    state_homogeneous(4)=1.0;

    // Apply transformation
    state_homogeneous=T_homogeneous * state_homogeneous;

    // Convert back to state coordinates
    state(1)=state_homogeneous(1);   state(2)=state_homogeneous(2);  state(3)=state_homogeneous(3);


    // sample from additive noise
    Sample<ColumnVector> noise;
    _additiveNoise.SampleFrom(noise, method, args);

    // store results in one_sample
    one_sample.ValueSet(state+noise.ValueGet());


    return true;
}
void NonlinearSystemPdf::AdditiveNoiseSigmaSet(const MatrixWrapper::SymmetricMatrix &sigma)
{
    _additiveNoise.CovarianceSet(sigma);
}


}//namespace BFL

