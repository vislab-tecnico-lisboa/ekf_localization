// $Id: nonlinearanalyticconditionalgaussianmobile.h 5374 2005-05-06 14:57:05Z TDeLaet $
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


#ifndef __NON_LINEAR_SYSTEM_CONDITIONAL_GAUSSIAN_MOBILE__
#define __NON_LINEAR_SYSTEM_CONDITIONAL_GAUSSIAN_MOBILE__

#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{
/// Non Linear Conditional Gaussian
/**
     - \f$ \mu = Matrix[1] . ConditionalArguments[0] +
     Matrix[2]. ConditionalArguments[1]  + ... + Noise.\mu \f$
     - Covariance is independent of the ConditionalArguments, and is
     the covariance of the Noise pdf
  */
class NonLinearAnalyticConditionalGaussianMobile : public AnalyticConditionalGaussianAdditiveNoise
{
public:
    /// Constructor
    /** @pre:  Every Matrix should have the same amount of rows!
      This is currently not checked.  The same goes for the number
      of columns, which should be equal to the number of rows of
      the corresponding conditional argument!
      @param ratio: vector containing the different matrices of
      the linear relationship between the conditional arguments
      and \f$\mu\f$
      @param additiveNoise Pdf representing the additive Gaussian uncertainty
      */
    NonLinearAnalyticConditionalGaussianMobile( const Gaussian& additiveNoise);

    /// Destructor
    virtual ~NonLinearAnalyticConditionalGaussianMobile();

    // redefine virtual functions
    virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
    virtual MatrixWrapper::Matrix          dfGet(unsigned int i)       const;
};

} // End namespace BFL

#endif //// $Id: nonlinearanalyticconditionalgaussianmobile.cpp 5823 2005-10-27 13:43:02Z TDeLaet $
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

#include "bayesian_filtering/nonlinearanalyticconditionalgaussianmobile.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng
// libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
using namespace MatrixWrapper;


NonLinearAnalyticConditionalGaussianMobile::NonLinearAnalyticConditionalGaussianMobile(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE)
{}


NonLinearAnalyticConditionalGaussianMobile::~NonLinearAnalyticConditionalGaussianMobile(){}
void angleOverflowCorrect(double& a)
{
    while ((a) >  M_PI) a -= 2*M_PI;
    while ((a) < -M_PI) a += 2*M_PI;
}
ColumnVector NonLinearAnalyticConditionalGaussianMobile::ExpectedValueGet() const
{
    // Input rot_1, trans, rot_2
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector delta_odom  = ConditionalArgumentGet(1)+AdditiveNoiseMuGet();
    state(1) += cos(state(3)+delta_odom(1))*delta_odom(2);
    state(2) += sin(state(3)+delta_odom(1))*delta_odom(2);
    state(3) += delta_odom(1)+delta_odom(3);
    angleOverflowCorrect(state(3));
    return state;
}

Matrix NonLinearAnalyticConditionalGaussianMobile::dfGet(unsigned int i) const
{
    if (i==0)//derivative to the second conditional argument (u)
    {
        ColumnVector state = ConditionalArgumentGet(0);
        ColumnVector delta_odom = ConditionalArgumentGet(1);
        //std::cout << "state:"<< state << std::endl;

        Matrix df(3,3);
//        df(1,1)=-sin(state(3)+delta_odom(1))*delta_odom(2);
//        df(1,2)=cos(state(3)+delta_odom(1));
//        df(2,2)=0;

//        df(2,1)=cos(state(3)+delta_odom(1))*delta_odom(2);
//        df(2,2)=sin(state(3)+delta_odom(1));
//        df(2,3)=0;

//        df(3,1)=1;
//        df(3,2)=0;
//        df(3,3)=1;


        df(1,1)=1;
        df(1,2)=0;
        df(1,3)=-sin(state(3)+delta_odom(1))*delta_odom(2);
        df(2,1)=0;
        df(2,2)=1;
        df(2,3)=cos(state(3)+delta_odom(1))*delta_odom(2);
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
        else{
            cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
            exit(-BFL_ERRMISUSE);
        }
    }
}

}//namespace BFL

